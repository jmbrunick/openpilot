cd /workspace/sr2
cfgs=""
cfgs="g35_0.88 g40_0.88 g45_0.88 g56_0.88 g35_0.85 g35_0.90 g30_0.90"
cfgs="$cfgs clash solocls noextra nohead"
( for c in $cfgs; do for g in $(cat segs.txt); do echo "$c $g"; done; done ) | xargs -P8 -L1 sh -c '
c=$0; mkdir -p out/v23$c; E=""
case $c in g*) px=${c#g}; px=${px%_*}; cf=${c#*_}; E="TS_SOLO_HI_PX=$px TS_SOLO_HI_CONF=$cf";;
 clash) E="TS_CLASH_VOTES=1";; solocls) E="TS_SOLO_CLASS_ONLY=1";; noextra) E="TS_EXTRA_READS=0";; nohead) E="TS_PROPOSER_BURST_HEADROOM=0.0";; esac
env $E OUT=v23$c TARGET=40 python3 sim23.py v23 $1 >> sw23_$c.err 2>&1'
echo SW23DONE
