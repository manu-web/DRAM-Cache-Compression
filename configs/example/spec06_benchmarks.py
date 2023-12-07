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
gcc.cmd = [gcc.executable] + ['scilab.i', '-o', 'scilab.s']
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
omnetpp.cmd = [omnetpp.executable] + ['omnetpp.ini']
#omnetpp.output = out_dir + 'omnetpp.out'

#450.soplex
soplex = Process() # Update June 7, 2017: This used to be LiveProcess()
bench_dir = spec2006_dir + 'soplex/'
soplex.executable = bench_dir + 'soplex' + x86_suffix
# TEST CMDS
#soplex.cmd = [soplex.executable] + ['-m10000', 'test.mps']
# REF CMDS
soplex.cmd = [soplex.executable] + ['-s1', '-e', '-m45000', 'pds-50.mps']
#soplex.cmd = [soplex.executable] + ['-m3500', 'ref.mps']
#soplex.output = out_dir + 'soplex.out'