#!/usr/bin/env bash
# Repeated OA-SLAM RGB-D runs with the YOLOv8n-seg TensorRT FP16 front end on Jetson,
# in the same pattern as run_slam_repeat_benchmark_ORB_SLAM3.sh (repetition-major).
#
# Usage: scripts/run_benchmark_trt.sh [first_rep] [last_rep] [seq_short ...]
#   default: repetitions 1..5, all 12 sequences below
# Layout: $RESULTS/OA-SLAM-Yolov8nSegFP16_<i>/<tum|bonn>/<short_seq>/
#   statistics.txt (ORB-SLAM3 format + loop markers), tegrastats.log, frame_times_*.csv,
#   camera_poses_*_tum.txt, evo_ape.txt, evo_rpe_trans.txt, evo_rpe_rot.txt, computation_cost.txt
# A run that stops before the last frame (crash) is moved to $RESULTS/_incomplete/ and the
# same slot is repeated, at most MAX_ATTEMPTS times; complete runs are never repeated.
# VIEWER=1 shows the Pangolin windows on the monitor attached to the Jetson (display :0).
# Power rails need root: tegrastats runs with `sudo -n` (NOPASSWD entry for /usr/bin/tegrastats).

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA="$HOME/Riset_SLAM/Dataset"
METHOD="OA-SLAM-Yolov8nSegFP16"
RESULTS="${RESULTS:-$HOME/Riset_SLAM/Results/repeat_experiment/$METHOD}"
ENGINE="$ROOT/models/yolov8n-seg_fp16.engine"
VIEWER="${VIEWER:-1}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-3}"
COOLDOWN="${COOLDOWN:-30}"
FIRST="${1:-1}"
LAST="${2:-5}"
shift 2 2>/dev/null

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
mkdir -p "$RESULTS/_incomplete"

{
    echo "date         : $(date -Iseconds)"
    echo "repetitions  : $FIRST..$LAST"
    echo "engine       : $ENGINE ($(md5sum "$ENGINE" | cut -c1-32))"
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
    echo "--- EXPERIMENT REPETITION [$i] ---"
    for ITEM in "${SEQS[@]}"; do
        IFS=":" read -r SHORT DATASET FOLDER CAMERA <<< "$ITEM"
        if [ "${#ONLY[@]}" -gt 0 ] && [[ ! " ${ONLY[*]} " =~ " $SHORT " ]]; then continue; fi
        SEQ="$DATA/$DATASET/$FOLDER"
        NFRAMES="$(grep -c . "$SEQ/associations.txt")"
        RUN_DIR="$RESULTS/${METHOD}_${i}/$DATASET/$SHORT"

        if is_complete "$RUN_DIR" "$SHORT" "$NFRAMES"; then
            echo "[$(date +%T)] skip $SHORT run $i (already complete)"; continue
        fi

        for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
            rm -rf "$RUN_DIR"; mkdir -p "$RUN_DIR"
            echo "[$(date +%T)] Processing $DATASET: $SHORT run $i (attempt $attempt, $NFRAMES frames, $CAMERA)"
            sudo -n tegrastats --interval 100 --logfile "$RUN_DIR/tegrastats.log" --start
            (cd "$RUN_DIR/.." && exec "$ROOT/bin/oa-slam_rgb-d-offline" \
                "$ROOT/Vocabulary/ORBvoc.txt" "$ROOT/Cameras/$CAMERA" "$SEQ" "$SEQ/associations.txt" \
                "$ENGINE" null points+objects "$SHORT" "$VIEWER") > "$RUN_DIR/stdout.log" 2>&1
            STATUS=$?
            sudo -n tegrastats --stop
            echo "$STATUS" > "$RUN_DIR/exit_code.txt"

            if is_complete "$RUN_DIR" "$SHORT" "$NFRAMES"; then
                [ "$STATUS" -ne 0 ] && echo "  note: exit $STATUS after the last frame (run is complete)"
                break
            fi
            DONE_FRAMES=$(($(wc -l < "$RUN_DIR/frame_times_$SHORT.csv" 2>/dev/null || echo 1) - 1))
            DEST="$RESULTS/_incomplete/${METHOD}_${i}/$DATASET/${SHORT}_att${attempt}_$(date +%H%M%S)"
            mkdir -p "$(dirname "$DEST")"; mv "$RUN_DIR" "$DEST"
            printf "%s\t%s\trun%s\tattempt%s\texit=%s\tframes=%s/%s\t%s\n" "$(date -Iseconds)" "$SHORT" "$i" \
                "$attempt" "$STATUS" "$DONE_FRAMES" "$NFRAMES" "$DEST" | tee -a "$RESULTS/INCOMPLETE_ATTEMPTS.tsv"
            sleep "$COOLDOWN"
        done

        if is_complete "$RUN_DIR" "$SHORT" "$NFRAMES"; then
            GT="$SEQ/groundtruth.txt"; EST="$RUN_DIR/camera_poses_${SHORT}_tum.txt"
            evo_ape tum "$GT" "$EST" -a --no_warnings > "$RUN_DIR/evo_ape.txt" 2>&1
            evo_rpe tum "$GT" "$EST" -a -r trans_part --delta 1 --delta_unit f --no_warnings > "$RUN_DIR/evo_rpe_trans.txt" 2>&1
            evo_rpe tum "$GT" "$EST" -a -r angle_deg --delta 1 --delta_unit f --no_warnings > "$RUN_DIR/evo_rpe_rot.txt" 2>&1
            python3 "$ROOT/scripts/summarize_run.py" "$RUN_DIR" "$SHORT" "${METHOD}_${i}" > /dev/null
            sed -n '4,13p' "$RUN_DIR/computation_cost.txt"
        else
            echo "  FAILED: $SHORT run $i incomplete after $MAX_ATTEMPTS attempts" | tee -a "$RESULTS/errors.log"
        fi
        sleep "$COOLDOWN"   # let the SoC cool down between runs
    done
done

python3 "$ROOT/scripts/tabulate_results.py" "$RESULTS"
echo "All benchmarks finished at $(date)"
