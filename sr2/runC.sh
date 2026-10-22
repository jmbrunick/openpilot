cd /workspace/sr2
( for c in p34 p36 p40; do for g in $(cat segs_all.txt); do echo "$c $g"; done; done ) | xargs -P8 -L1 sh -c '
c=$0; E="TS_EXTRA_ON_SILENT=1 TS_SOLO_CLASS_CONF=0.90 TS_PAIR_MIN_PX=${c#p}"
[ -f out/vC$c/$1.json ] && exit 0
env $E OUT=vC$c TARGET=40 python3 sim23.py v23 $1 >> qC_$c.err 2>&1'
echo CDONE
./backup.sh
