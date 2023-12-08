TOPDIR = $(PWD)

OUTDIR = $(TOPDIR)/out_dir
BENCHMARKS = $(TOPDIR)/benchmarks

SIMPDIR = $(TOPDIR)/SimPoint.3.2
SIMPBIN = $(SIMPDIR)/bin/simpoint

BENCH = saxpy
GEM5OUT = $(BENCH)
BENCHTOPDIR = $(OUTDIR)/$(GEM5OUT)
 
ifdef DBG_FLAGS
	DBG = --debug-flags
	DBG += $(DBG_FLAGS)
endif

MAXINSTS = 5000000000
WARMPUP = 100000000
CPU_TYPE = X86O3CPU
SIMP_FLAGS =
COMMON_CONFIG_ARGS = 
DC = 0
BBV = 0
CHKP = 0

L1CACHE = --caches --l1d_size='64kB' --l1i_size='16kB'
L2CACHE = --l2cache --l2_size='256kB'

CHKP_IDX = 1

BBVDIR = $(BENCHTOPDIR)/bbv
CHKPDIR = $(BENCHTOPDIR)/chkpt

SIMPOINTS = $(BENCHTOPDIR)/$(BENCH).simpoint
SIMPWEIGHTS = $(BENCHTOPDIR)/$(BENCH).weight

SIMP_INT = 100000000
ifeq ($(BBV), 1)
	SIMP_FLAGS += --simpoint-profile
	SIMP_FLAGS += --simpoint-interval $(SIMP_INT)
	CPU_TYPE := NonCachingSimpleCPU
	DRAM_CACHE = False
	DUMPDIR = $(BBVDIR)
else ifeq ($(CHKP), 1)
	SIMP_FLAGS += --take-simpoint-checkpoint=$(SIMPOINTS),$(SIMPWEIGHTS),$(SIMP_INT),$(WARMPUP)
	CPU_TYPE := TimingSimpleCPU
	DRAM_CACHE = False
	DUMPDIR = $(CHKPDIR)
else
	ifeq ($(DC), 1)
		DRAM_CACHE = True
		DUMPDIR = $(BENCHTOPDIR)/dc
		COMMON_CONFIG_ARGS += --dram-cache $(DRAM_CACHE)
	else
		DRAM_CACHE = False
		DUMPDIR = $(BENCHTOPDIR)/base
	endif
	SIMP_FLAGS += --restore-simpoint-checkpoint -r $(CHKP_IDX) --checkpoint-dir $(CHKPDIR) 
endif

COMMON_CONFIG_ARGS += --cpu-type=$(CPU_TYPE) $(L1CACHE) $(L2CACHE) --mem-size '8GiB'

simulate:
	build/X86/gem5.opt $(DBG) --outdir=$(DUMPDIR) configs/example/dram_cache.py $(COMMON_CONFIG_ARGS) --benchmark $(BENCH) --benchmark-stdout $(DUMPDIR)/$(BENCH).out --benchmark-stderr $(DUMPDIR)/$(BENCH).err $(SIMP_FLAGS) >| $(OUTDIR)/$(BENCH)_sim.out
	mv $(OUTDIR)/$(BENCH)_sim.out $(DUMPDIR)/

simpoint:
	$(SIMPBIN) -loadFVFile $(BENCHTOPDIR)/bbv/simpoint.bb -maxK 30 -saveSimpoints $(SIMPOINTS) -saveSimpointWeights $(SIMPWEIGHTS) 

valgrind:
	valgrind --tool=exp-bbv $(CMD) --bb-out-file=$(BENCHTOPDIR)/bbv/$(BENCH).bb

simulate_test:
	build/X86/gem5.opt $(DBG) --outdir=$(DUMPDIR) configs/example/dram_cache.py $(COMMON_CONFIG_ARGS) --cmd=$(BENCHMARKS)/$(BENCH)/$(BENCH).exe $(SIMP_FLAGS) >| $(OUTDIR)/$(BENCH)_sim.out

simpoint_test:
	$(SIMPBIN) -loadFVFile $(DUMPDIR)/simpoint.bb.gz -maxK 30 -saveSimpoints $(SIMPOINTS) -saveSimpointWeights $(SIMPWEIGHTS) -inputVectorsGzipped

checkpoint_test:
	build/X86/gem5.opt $(DBG) --outdir=$(DUMPDIR)/chkpt configs/example/dram_cache.py $(COMMON_CONFIG_ARGS) --cmd=$(BENCHMARKS)/$(BENCH)/$(BENCH).exe $(CHKP_FLAGS) >| $(OUTDIR)/$(BENCH)_sim.out

saxpy:
	g++ -march=native -O0 $(BENCHMARKS)/saxpy/saxpy.cpp -o $(BENCHMARKS)/saxpy/saxpy.exe

all_test:
	make saxpy
	make simulate_test
	make simpoint_test