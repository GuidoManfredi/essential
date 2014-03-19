#!/usr/bin/perl
##############################################################
# A Perl Batch to spawn blender renderings                   #
# (C) Sept. 2002 Stefano (S68) Selleri                       #
# V. 0.0.1                                                   #
##############################################################
####               PARAMETERS TO CONFIGURE                ####
@NodeNames   = ("Node0","Node1","Node2");
#### The above parameter specifies the names of the machines (nodes)
#    onto which the rendering will be spawned.
#    spawning is made via 'rsh NodeName Command' so rsh must be 
#    installed and working
#
#
@ProcessXNode = (1,1,4);
#### Number of processes for each node. 
#    1 is best choiche unless the node has multiple CPUs.
#    In this case as many processes as there are CPUs is OK
#
#
$RootDir   = "/nfs/home/myhome/blender";
#### The directory where the .blend file is and where the images
#    and logs will be store. It is the SAME for all nodes so two
#    possibilities:
#    1 - All nodes has this dir in their path and you make
#        multiple copies of the .blend file and you'll have
#        your rendering scattered all over the cluster
#    2 - (preferred) a directory is shared on one node and visible
#        by all other, in this directory a single copy of the .blend file
#        is stored and output.
#
#    The rendering will be produced independently, hence if you 
#    chosen AVI file you'll have as many AVIs as processes.
#    rendering TARGAs or JPGs and then using the sequence
#    editor can be better.
#
#    Console Outputs by Blender are redirected on a file, in
#    the RootDir directory, named dumpblend.N.M where N is the node
#    number (0,1,...) and M is the process number in that node,
#    usefull for debugging
#
#    In the directory where the blendbat.pl ils launched other logs
#    are created. A dumpproc.N.M holding the command line output
#    of the rsh command (numbered as before) and a bblog file
#    holding the output of the Perl system call to rsh.
#    If everithing runs smoothly these files are empty.
# 
#
####                END OF CONFIG SECTION                 ####
#### Do not edit below unless you know what you are doing ####

$f = $ARGV[0] || die "Missing file name (try blendbat.pl -h)\n";
if ($f=~/^-h/){
  print "\n\n  blendbat.pl (C) Sept. 2002 Stefano (S68) Selleri\n\n";
  print "usage:\n\n     blendbat.pl file.blend StartFrame EndFrame\n\n";
  print "  flile.blend - The blender file. it must be located in\n";
  print "                directory specified in the configuration parameters\n";
  print "                of this script.i\n\n";
  print "  StartFrame  - First frame of the batch\n";
  print "  EndFrame    - Last frame of the batch\n\n";
  die;
}
   
$s = $ARGV[1] || die "Missing start frame (try blendbat.pl -h)\n";
$e = $ARGV[2] || die "Missing end frame (try blendbat.pl -h)\n";

$totp = 0;foreach $np (@ProcessXNode) {$totp+=$np}

$nf=$e-$s; 
$nfpp=int($nf/$totp);

$s[0]=$s;$e[0]=$s+$nfpp-1;
for($i=1;$i<$totp;$i++) {
  $s[$i]=$e[$i-1]+1;$e[$i]=$s[$i]+$nfpp-1;
}
if ($e[$totp-1]<$e){$e[$totp-1]=$e;}

$idx=0;
for ($i=0;$i<@NodeNames;$i++) {
  for ($j=0;$j<$ProcessXNode[$i];$j++) {
    print "+--+--> Spawning rendering of frames $s[$idx] to $e[$idx] \n";
    print "|  +--> on node $i ($NodeNames[$i])\n";
    print "|  +--> this is process numbe $j on this node\n";
    print "|\n";
  
    $process[$idx] =  `rsh $NodeNames[$i] "cd $RootDir ; blender -b $f -s $s[$idx] -e $e[$idx] -a > dumpblend.$i.$j" >dumpproc.$i.$j &`;
    $idx++;
  }
}

open (FL, ">bblog");
print FL join ("\n",@process);
close(FL);



