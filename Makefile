TOPDIR = $(PWD)

OUTDIR = $(TOPDIR)/out_dir
BENCHMARKS = $(TOPDIR)/benchmarks

ifdef DBG_FLAGS
	DBG = --debug-flags
	DBG += $(DBG_FLAGS)
endif


CPU_TYPE = X86O3CPU

DC = True

GEM5OUT = $(BENCH)

sim_bench:
	build/X86/gem5.opt $(DBG) --outdir=$(OUTDIR)/$(GEM5OUT) configs/example/dram_cache.py --cpu-type=$(CPU_TYPE) --caches --benchmark $(BENCH) --benchmark-stdout $(OUTDIR)/$(BENCH).out --benchmark-stderr $(OUTDIR)/$(BENCH).err --dram-cache $(DC) >| $(OUTDIR)/$(BENCH)_$(DC).out

sim_test:
	build/X86/gem5.opt $(DBG) --outdir=$(OUTDIR)/$(GEM5OUT)_$(DC) configs/example/dram_cache.py --cpu-type=$(CPU_TYPE) --caches --cmd=$(BENCHMARKS)/saxpy/saxpy.exe --dram-cache $(DC) >| $(OUTDIR)/saxpy_$(DC).out