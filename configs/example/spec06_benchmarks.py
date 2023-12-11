import m5
from m5.objects import *
 
# These three directory paths are not currently used.
#gem5_dir = '<FULL_PATH_TO_YOUR_GEM5_INSTALL>'
#spec_dir = '<FULL_PATH_TO_YOUR_SPEC_CPU2006_INSTALL>'
#out_dir = '<FULL_PATH_TO_DESIRED_OUTPUT_DIRECTORY>'
 
x86_suffix = '_base.x86_64_sse'
spec2006_dir = '/nobackup/aatmanb/DRAM-Cache-Compression/benchmarks/spec2006/'

#403.gcc
gcc = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'gcc/' 
gcc.executable = bench_dir + 'gcc' + x86_suffix
# TEST CMDS
#gcc.cmd = [gcc.executable] + ['cccp.i', '-o', 'cccp.s']
# REF CMDS
#gcc.cmd = [gcc.executable] + ['166.i', '-o', '166.s']
#gcc.cmd = [gcc.executable] + ['200.i', '-o', '200.s']
#gcc.cmd = [gcc.executable] + ['c-typeck.i', '-o', 'c-typeck.s']
#gcc.cmd = [gcc.executable] + ['cp-decl.i', '-o', 'cp-decl.s']
#gcc.cmd = [gcc.executable] + ['expr.i', '-o', 'expr.s']
#gcc.cmd = [gcc.executable] + ['expr2.i', '-o', 'expr2.s']
#gcc.cmd = [gcc.executable] + ['g23.i', '-o', 'g23.s']
#gcc.cmd = [gcc.executable] + ['s04.i', '-o', 's04.s']
gcc.cmd = [gcc.executable] + [bench_dir + 'input/scilab.i', '-o', bench_dir + 'scilab.s']
#gcc.output = out_dir + 'gcc.out'

#437.leslie3d
leslie3d = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'leslie3d/'
leslie3d.executable = bench_dir + 'leslie3d' + x86_suffix
# TEST CMDS
#leslie3d.cmd = [leslie3d.executable]
#leslie3d.input = 'leslie3d.in'
# REF CMDS
leslie3d.cmd = [leslie3d.executable]
leslie3d.input = bench_dir + 'leslie3d.in'
#leslie3d.output = bench_dir + 'leslie3d.out'

#462.libquantum
libquantum = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'libquantum/'
libquantum.executable = bench_dir + 'libquantum' + x86_suffix
# TEST CMDS
#libquantum.cmd = [libquantum.executable] + ['33','5']
# REF CMDS [UPDATE 10/2/2015]: Sparsh Mittal has pointed out the correct input for libquantum should be 1397 and 8, not 1297 and 8. Thanks!
libquantum.cmd = [libquantum.executable] + ['1397','8']
#libquantum.output = out_dir + 'libquantum.out'

#471.omnetpp
omnetpp = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'omnetpp/'
omnetpp.executable = bench_dir + 'omnetpp' + x86_suffix 
# TEST CMDS
#omnetpp.cmd = [omnetpp.executable] + ['omnetpp.ini']
# REF CMDS
omnetpp.cmd = [omnetpp.executable] + [bench_dir + 'omnetpp.ini']
#omnetpp.output = out_dir + 'omnetpp.out'

#450.soplex
soplex = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'soplex/'
soplex.executable = bench_dir + 'soplex' + x86_suffix
# TEST CMDS
#soplex.cmd = [soplex.executable] + ['-m10000', 'test.mps']
# REF CMDS
soplex.cmd = [soplex.executable] + ['-s1', '-e', '-m45000', bench_dir + 'pds-50.mps']
#soplex.cmd = [soplex.executable] + ['-m3500', 'ref.mps']
#soplex.output = out_dir + 'soplex.out'

#400.perlbench
perlbench = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'perlbench/'
perlbench.executable = bench_dir + 'perlbench' + x86_suffix
# TEST CMDS
#perlbench.cmd = [perlbench.executable] + ['-I.', '-I./lib', 'attrs.pl']
# REF CMDS
perlbench.cmd = [perlbench.executable] + ['-I'+bench_dir+'lib', 'checkspam.pl', '2500', '5', '25', '11', '150', '1', '1', '1', '1']
#perlbench.cmd = [perlbench.executable] + ['-I./lib', 'diffmail.pl', '4', '800', '10', '17', '19', '300']
#perlbench.cmd = [perlbench.executable] + ['-I./lib', 'splitmail.pl', '1600', '12', '26', '16', '4500']
#perlbench.output = out_dir+'perlbench.out'
 
#429.mcf
mcf = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'mcf/'
mcf.executable = bench_dir + 'mcf' + x86_suffix
# TEST CMDS
#mcf.cmd = [mcf.executable] + ['inp.in']
# REF CMDS
mcf.cmd = [mcf.executable] + [bench_dir + 'inp.in']
#mcf.output = out_dir + 'mcf.out'
 
#433.milc
milc = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'milc/'
milc.executable = bench_dir +'milc' + x86_suffix
# TEST CMDS
#milc.cmd = [milc.executable]
#milc.input = 'su3imp.in'
# REF CMDS
milc.cmd = [milc.executable]
milc.input = bench_dir + 'input/su3imp.in'
#milc.output = out_dir + 'milc.out'
 
#459.GemsFDTD
GemsFDTD = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'GemsFDTD/'
GemsFDTD.executable = bench_dir +'GemsFDTD' + x86_suffix 
# TEST CMDS
#GemsFDTD.cmd = [GemsFDTD.executable]
# REF CMDS
GemsFDTD.cmd = [GemsFDTD.executable]
#GemsFDTD.output = out_dir + 'GemsFDTD.out'

#470.lbm
lbm = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'lbm/'
lbm.executable = bench_dir +'lbm' + x86_suffix
# TEST CMDS
#lbm.cmd = [lbm.executable] + ['20', 'reference.dat', '0', '1', '100_100_130_cf_a.of']
# REF CMDS
lbm.cmd = [lbm.executable] + ['300', bench_dir+'reference.dat', '0', '0', bench_dir+'100_100_130_ldc.of']
#lbm.output = out_dir + 'lbm.out'