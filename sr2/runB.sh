cd /workspace/sr2
while ! grep -q QDONE runq.out; do sleep 30; done
( for c in b4sil s85 s90 s93 s93h45; do for g in $(cat segs_all.txt); do echo "$c $g"; done; done ) | xargs -P8 -L1 sh -c '
c=$0; E="TS_EXTRA_ON_SILENT=1"
case $c in b4sil) E="$E TS_BURST_READS=4";; s85) E="$E TS_SOLO_CLASS_CONF=0.85";; s90) E="$E TS_SOLO_CLASS_CONF=0.90";; s93) E="$E TS_SOLO_CLASS_CONF=0.93";; s93h45) E="$E TS_SOLO_CLASS_CONF=0.93 TS_SOLO_CLASS_PX=45";; esac
[ -f out/vB$c/$1.json ] && exit 0
env $E OUT=vB$c TARGET=40 python3 sim23.py v23 $1 >> qB_$c.err 2>&1'
echo BDONE
./backup.sh
