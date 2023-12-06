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

SIMP_FLAGS =
SIMP_INT = 1000000

ifeq ($(SIMP), 1)
	SIMP_FLAGS += --simpoint-profile
	SIMP_FLAGS += --simpoint-interval $(SIMP_INT)
	CPU_TYPE := NonCachingSimpleCPU
endif

SIMPDIR = $(TOPDIR)/SimPoint.3.2
SIMPBIN = $(SIMPDIR)/bin/simpoint

BENCH = saxpy

sim_bench:
	build/X86/gem5.opt $(DBG) --outdir=$(OUTDIR)/$(GEM5OUT) configs/example/dram_cache.py --cpu-type=$(CPU_TYPE) --caches --benchmark $(BENCH) --benchmark-stdout $(OUTDIR)/$(BENCH).out --benchmark-stderr $(OUTDIR)/$(BENCH).err --dram-cache $(DC) $(SIMP_FLAGS) >| $(OUTDIR)/$(BENCH).out

simp_bench:
	$(SIMPBIN) -loadFVFile $(OUTDIR)/$(GEM5OUT)/simpoint.bb.gz -maxK 30 -saveSimpoints $(OUTDIR)/$(GEM5OUT)/$(BENCH).simpoint -saveSimpointWeights $(OUTDIR)/$(GEM5OUT)/$(BENCH).weight -inputVectorsGzipped

all_bench:
	make sim_bench
	make simp_bench

sim_test:
	build/X86/gem5.opt $(DBG) --outdir=$(OUTDIR)/$(GEM5OUT) configs/example/dram_cache.py --cpu-type=$(CPU_TYPE) --caches --cmd=$(BENCHMARKS)/$(BENCH)/$(BENCH).exe --dram-cache $(DC) $(SIMP_FLAGS) >| $(OUTDIR)/$(BENCH).out

simp_test:
	$(SIMPBIN) -loadFVFile $(OUTDIR)/$(GEM5OUT)/simpoint.bb.gz -maxK 30 -saveSimpoints $(OUTDIR)/$(GEM5OUT)/$(BENCH).simpoint -saveSimpointWeights $(OUTDIR)/$(GEM5OUT)/$(BENCH).weight -inputVectorsGzipped

saxpy:
	g++ -march=native -O0 $(BENCHMARKS)/saxpy/saxpy.cpp -o $(BENCHMARKS)/saxpy/saxpy.exe

all_test:
	make saxpy
	make sim_test
	make simp