#!/bin/bash
# =============================================================================
# run_all_figures.sh
# Runs all flutter analysis figure scripts in sequence.
# =============================================================================

set -e  # Stop on first error

OL_RDM=~/Desktop/Flutter_analysis/figures_for_dissertation/off_frequency_tests/OL_RDM
OFF_FREQ=~/Desktop/Flutter_analysis/figures_for_dissertation/off_frequency_tests

CONDITIONS="CLOSED_LOOP OPEN_LOOP_55 POISSON UNIFORM_JITTER PHASE_RANDOMIZED REPLAY"

echo "============================================================"
echo "  Flutter Figure Generation"
echo "  $(date)"
echo "============================================================"

# 1. All conditions comparison + phase coherence
echo ""
echo "[1/5] plot_CL_RDM.py — All conditions vs closed loop"
cd "$OFF_FREQ"
python3 plot_CL_RDM.py

# 2. CL vs Replay
echo ""
echo "[2/5] plot_cl_vs_replay.py — Live vs replay"
cd "$OL_RDM"
python3 plot_cl_vs_replay.py

# 3. FFT energy comparison
echo ""
echo "[3/5] plot_fft_energy.py — Spectral energy"
cd "$OL_RDM"
python3 plot_fft_energy.py

# 4. FFT with reduction overlay
echo ""
echo "[4/5] plot_fft_with_reduction.py — Control effectiveness vs frequency"
cd "$OFF_FREQ"
python3 plot_fft_with_reduction.py

# 5. Phase-triggered average — all 6 conditions
echo ""
echo "[5/5] plot_phase_triggered_avg.py — Phase-triggered average"
cd "$OL_RDM"
python3 plot_phase_triggered_avg.py \
    --bag-dir . \
    --output-dir . \
    --conditions $CONDITIONS

echo ""
echo "============================================================"
echo "  All figures complete — $(date)"
echo "============================================================"