# ── PMSM FOC Simulation — Makefile ──────────────────────────────────────────
CC      = gcc
CFLAGS  = -O2 -lm
TARGET  = pmsm_foc
SRC     = pmsm_foc.c

.PHONY: all run run-custom log clean help

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $<
	@echo "Built $(TARGET)"

# Default run: pipe directly to scope with default signals
run: $(TARGET)
	./$(TARGET) | python3 scope.py

# Custom signals example:
#   make run-custom VARS="omega_m_rpm omega_ref_rpm group: iq iq_ref group: Te TL"
run-custom: $(TARGET)
	./$(TARGET) | python3 scope.py $(VARS)

# Save to CSV and then plot from file
log: $(TARGET)
	./$(TARGET) > sim_log.csv
	@echo "Saved to sim_log.csv"
	python3 scope.py --file sim_log.csv

# Plot specific signals from saved log
plot: sim_log.csv
	python3 scope.py --file sim_log.csv $(VARS)

# List available variables
list:
	python3 scope.py --list

clean:
	rm -f $(TARGET) sim_log.csv

help:
	@echo ""
	@echo "  make            Build the simulation"
	@echo "  make run        Build + run with live scope (default signals)"
	@echo "  make run-custom VARS='...'  Run with custom signals"
	@echo "  make log        Save CSV, then open scope from file"
	@echo "  make plot VARS='...'  Plot custom signals from saved CSV"
	@echo "  make list       Show all available variable names"
	@echo ""
	@echo "  Example custom run:"
	@echo "    make run-custom VARS='omega_m_rpm omega_ref_rpm group: id id_ref'"
	@echo ""
