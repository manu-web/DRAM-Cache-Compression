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
gcc.executable = spec2006_dir + 'gcc/' + 'gcc' + x86_suffix
# TEST CMDS
#gcc.cmd = [gcc.executable] + ['cccp.i', '-o', 'cccp.s']
# REF CMDS
gcc.cmd = [gcc.executable] + ['166.i', '-o', '166.s']
#gcc.cmd = [gcc.executable] + ['200.i', '-o', '200.s']
#gcc.cmd = [gcc.executable] + ['c-typeck.i', '-o', 'c-typeck.s']
#gcc.cmd = [gcc.executable] + ['cp-decl.i', '-o', 'cp-decl.s']
#gcc.cmd = [gcc.executable] + ['expr.i', '-o', 'expr.s']
#gcc.cmd = [gcc.executable] + ['expr2.i', '-o', 'expr2.s']
#gcc.cmd = [gcc.executable] + ['g23.i', '-o', 'g23.s']
#gcc.cmd = [gcc.executable] + ['s04.i', '-o', 's04.s']
#gcc.cmd = [gcc.executable] + ['scilab.i', '-o', 'scilab.s']
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