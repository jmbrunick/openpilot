cd /workspace/sr2
while ! grep -q CDONE runC.out; do sleep 30; done
( for c in sp35 sp35p34 sp40; do for g in $(cat segs_all.txt); do echo "$c $g"; done; done ) | xargs -P8 -L1 sh -c '
c=$0; E="TS_EXTRA_ON_SILENT=1 TS_SOLO_CLASS_CONF=0.90"
case $c in sp35) E="$E TS_SOLO_CLASS_PX=35";; sp35p34) E="$E TS_SOLO_CLASS_PX=35 TS_PAIR_MIN_PX=34";; sp40) E="$E TS_SOLO_CLASS_PX=40";; esac
[ -f out/vD$c/$1.json ] && exit 0
env $E OUT=vD$c TARGET=40 python3 sim23.py v23 $1 >> qD_$c.err 2>&1'
echo DDONE
./backup.sh
