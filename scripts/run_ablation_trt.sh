#!/usr/bin/env bash
# Object-association ablation of OA-SLAM RGB-D with the YOLOv8n-seg TensorRT FP16 (640x640) front end on Jetson.
# 9 fixed cells x 12 sequences x 3 repetitions, same sequences, camera files and run protocol as
# run_benchmark_trt.sh (repeated run). Order: repetition -> sequence -> cell.
#
# Usage: scripts/run_ablation_trt.sh [first_rep] [last_rep] [seq_short ...]
#   default: repetitions 1..3, all 12 sequences
# Layout: $RESULTS/<cell>/${METHOD}_<cell>_<i>/<tum|bonn>/<short_seq>/  (same files as the repeated run)
# Cells: OA_IOU2D / OA_IOU3D / OA_MIN_OBS are read by src/Tracking.cc (printed as EFFECTIVE ... in stdout.log);
# "person excluded" drops COCO class 0 in the detector (scripts/ignore_person.txt), so no person mask is built.
# Complete runs are skipped, so the same command resumes an interrupted campaign.
# VIEWER=1 shows the Pangolin windows on the monitor attached to the Jetson (display :0).

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA="$HOME/Riset_SLAM/Dataset"
METHOD="${METHOD:-OA-SLAM3-Yolov8nSegFP16}"  # ORB-SLAM3 backbone (ORB-SLAM2 runs: OA-SLAM-Yolov8nSegFP16)
RESULTS="${RESULTS:-$HOME/Riset_SLAM/Results/ablations/${METHOD}_ablation}"
ENGINE="$ROOT/models/yolov8n-seg_fp16.engine"
VIEWER="${VIEWER:-1}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-3}"
COOLDOWN="${COOLDOWN:-30}"
FIRST="${1:-1}"
LAST="${2:-3}"
shift 2 2>/dev/null
IGNORE_PERSON="$ROOT/scripts/ignore_person.txt"

# cell : OA_IOU2D : OA_IOU3D : OA_MIN_OBS : ignore file (null = person used)
CELLS=(
    "default:0.20:0.30:10:null"
    "iou_0.10_0.15:0.10:0.15:10:null"
    "iou_0.40_0.50:0.40:0.50:10:null"
    "nobs5:0.20:0.30:5:null"
    "nobs20:0.20:0.30:20:null"
    "noperson:0.20:0.30:10:$IGNORE_PERSON"
    "iou_0.10_0.15_nobs5:0.10:0.15:5:null"
    "iou_0.10_0.15_nobs20:0.10:0.15:20:null"
    "iou_0.10_0.15_noperson:0.10:0.15:10:$IGNORE_PERSON"
)

# short name : dataset : folder : camera file
SEQS=(
    "fr3_sitting_xyz:tum:rgbd_dataset_freiburg3_sitting_xyz:TUM3_RGBD.yaml"
    "fr3_walking_half:tum:rgbd_dataset_freiburg3_walking_halfsphere:TUM3_RGBD.yaml"
    "fr3_walking_xyz:tum:rgbd_dataset_freiburg3_walking_xyz:TUM3_RGBD.yaml"
    "fr2_desk_w_person:tum:rgbd_dataset_freiburg2_desk_with_person:TUM2_RGBD.yaml"
    "fr3_walking_static:tum:rgbd_dataset_freiburg3_walking_static:TUM3_RGBD.yaml"
    "fr3_walking_rpy:tum:rgbd_dataset_freiburg3_walking_rpy:TUM3_RGBD.yaml"
    "crowd:bonn:rgbd_bonn_crowd:Bonn_RGBD.yaml"
    "person_track:bonn:rgbd_bonn_person_tracking:Bonn_RGBD.yaml"
    "crowd2:bonn:rgbd_bonn_crowd2:Bonn_RGBD.yaml"
    "fr1_desk2:tum:rgbd_dataset_freiburg1_desk2:TUM1_RGBD.yaml"
    "fr1_room:tum:rgbd_dataset_freiburg1_room:TUM1_RGBD.yaml"
    "fr2_pioneer_slam:tum:rgbd_dataset_freiburg2_pioneer_slam:TUM2_RGBD.yaml"
)
ONLY=("$@")

if [ "$VIEWER" = "1" ]; then
    export DISPLAY=:0
    export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
fi
if ! sudo -n tegrastats --help >/dev/null 2>&1; then
    echo "STOP: tegrastats needs root for the power rails (sudo -n tegrastats failed)"; exit 1
fi
if pgrep -x oa-slam_rgb-d-o >/dev/null; then
    echo "STOP: another oa-slam_rgb-d-offline process is running"; exit 1
fi
[ "$(cat "$IGNORE_PERSON")" = "0" ] || { echo "STOP: $IGNORE_PERSON must contain the single class id 0"; exit 1; }
mkdir -p "$RESULTS/_incomplete"
rm -f "$RESULTS/CAMPAIGN_DONE.txt"

{
    echo "date         : $(date -Iseconds)"
    echo "repetitions  : $FIRST..$LAST"
    echo "cells        : ${CELLS[*]}"
    echo "engine       : $ENGINE ($(md5sum "$ENGINE" | cut -c1-32))"
    echo "binary       : $(md5sum "$ROOT/bin/oa-slam_rgb-d-offline" | cut -c1-32), lib $(md5sum "$ROOT/lib/libOA_SLAM3.so" | cut -c1-32)"
    echo "power mode   : $(nvpmodel -q 2>/dev/null | head -1)"
    echo "jetpack      : $(head -1 /etc/nv_tegra_release)"
    echo "viewer       : $VIEWER"
    echo "max attempts : $MAX_ATTEMPTS per run"
    echo "git          : $(git -C "$ROOT" rev-parse --short HEAD) (+ local changes: $(git -C "$ROOT" status --short | wc -l) files)"
} | tee -a "$RESULTS/session_info.txt"

# A run is complete when every frame was tracked and the trajectory has one row per frame.
is_complete() {  # run_dir name n_frames
    local traj="$1/camera_poses_$2_tum.txt" times="$1/frame_times_$2.csv"
    [ -f "$traj" ] && [ -f "$times" ] && [ -f "$1/statistics.txt" ] &&
        [ "$(wc -l < "$traj")" -eq "$3" ] && [ "$(($(wc -l < "$times") - 1))" -eq "$3" ]
}

for i in $(seq "$FIRST" "$LAST"); do
    echo "--- ABLATION REPETITION [$i] ---"
    for ITEM in "${SEQS[@]}"; do
        IFS=":" read -r SHORT DATASET FOLDER CAMERA <<< "$ITEM"
        if [ "${#ONLY[@]}" -gt 0 ] && [[ ! " ${ONLY[*]} " =~ " $SHORT " ]]; then continue; fi
        SEQ="$DATA/$DATASET/$FOLDER"
        NFRAMES="$(grep -c . "$SEQ/associations.txt")"

        for CELL_ITEM in "${CELLS[@]}"; do
            IFS=":" read -r CELL IOU2D IOU3D MINOBS IGNORE <<< "$CELL_ITEM"
            LABEL="${METHOD}_${CELL}_${i}"
            RUN_DIR="$RESULTS/$CELL/$LABEL/$DATASET/$SHORT"

            if is_complete "$RUN_DIR" "$SHORT" "$NFRAMES"; then
                echo "[$(date +%T)] skip $SHORT $CELL run $i (already complete)"; continue
            fi

            for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
                rm -rf "$RUN_DIR"; mkdir -p "$RUN_DIR"
                echo "[$(date +%T)] Processing $DATASET: $SHORT cell $CELL run $i (attempt $attempt, $NFRAMES frames, $CAMERA," \
                     "IoU $IOU2D/$IOU3D, N_obs $MINOBS, ignore $(basename "$IGNORE"))"
                sudo -n tegrastats --interval 100 --logfile "$RUN_DIR/tegrastats.log" --start
                (cd "$RUN_DIR/.." && OA_IOU2D="$IOU2D" OA_IOU3D="$IOU3D" OA_MIN_OBS="$MINOBS" exec "$ROOT/bin/oa-slam_rgb-d-offline" \
                    "$ROOT/Vocabulary/ORBvoc.txt" "$ROOT/Cameras/$CAMERA" "$SEQ" "$SEQ/associations.txt" \
                    "$ENGINE" "$IGNORE" points+objects "$SHORT" "$VIEWER") > "$RUN_DIR/stdout.log" 2>&1
                STATUS=$?
                sudo -n tegrastats --stop
                echo "$STATUS" > "$RUN_DIR/exit_code.txt"

                if is_complete "$RUN_DIR" "$SHORT" "$NFRAMES"; then
                    [ "$STATUS" -ne 0 ] && echo "  note: exit $STATUS after the last frame (run is complete)"
                    break
                fi
                DONE_FRAMES=$(($(wc -l < "$RUN_DIR/frame_times_$SHORT.csv" 2>/dev/null || echo 1) - 1))
                DEST="$RESULTS/_incomplete/$CELL/$LABEL/$DATASET/${SHORT}_att${attempt}_$(date +%H%M%S)"
                mkdir -p "$(dirname "$DEST")"; mv "$RUN_DIR" "$DEST"
                printf "%s\t%s\t%s\trun%s\tattempt%s\texit=%s\tframes=%s/%s\t%s\n" "$(date -Iseconds)" "$SHORT" "$CELL" "$i" \
                    "$attempt" "$STATUS" "$DONE_FRAMES" "$NFRAMES" "$DEST" | tee -a "$RESULTS/INCOMPLETE_ATTEMPTS.tsv"
                sleep "$COOLDOWN"
            done

            if is_complete "$RUN_DIR" "$SHORT" "$NFRAMES"; then
                GT="$SEQ/groundtruth.txt"; EST="$RUN_DIR/camera_poses_${SHORT}_tum.txt"
                evo_ape tum "$GT" "$EST" -a --no_warnings > "$RUN_DIR/evo_ape.txt" 2>&1
                evo_rpe tum "$GT" "$EST" -a -r trans_part --delta 1 --delta_unit f --no_warnings > "$RUN_DIR/evo_rpe_trans.txt" 2>&1
                evo_rpe tum "$GT" "$EST" -a -r angle_deg --delta 1 --delta_unit f --no_warnings > "$RUN_DIR/evo_rpe_rot.txt" 2>&1
                python3 "$ROOT/scripts/summarize_run.py" "$RUN_DIR" "$SHORT" "$LABEL" > /dev/null
                grep -m1 "rmse" "$RUN_DIR/evo_ape.txt" | sed "s/^/  ATE /"
            else
                echo "  FAILED: $SHORT $CELL run $i incomplete after $MAX_ATTEMPTS attempts" | tee -a "$RESULTS/errors.log"
            fi
            sleep "$COOLDOWN"   # let the SoC cool down between runs
        done
    done
done

for CELL_ITEM in "${CELLS[@]}"; do
    CELL="${CELL_ITEM%%:*}"
    [ -d "$RESULTS/$CELL" ] && python3 "$ROOT/scripts/tabulate_results.py" "$RESULTS/$CELL"
done

N_DONE=$(find "$RESULTS" -path "$RESULTS/_incomplete" -prune -o -name metrics.json -print | wc -l)
MSG="OA-SLAM ablation FINISHED at $(date) -- $N_DONE complete runs, results in $RESULTS"
echo "$MSG" | tee "$RESULTS/CAMPAIGN_DONE.txt"
wall "$MSG" 2>/dev/null
DISPLAY=:0 XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority" notify-send "OA-SLAM ablation selesai" "$MSG" 2>/dev/null
echo "All ablation benchmarks finished at $(date)"
