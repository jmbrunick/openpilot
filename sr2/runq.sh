cd /workspace/sr2
# queue: config seg ; skips finished outputs
( for c in r21 d40 v23 early late b4 sil head fb2; do for g in $(cat segs_all.txt); do echo "$c $g"; done; done ) | xargs -P8 -L1 sh -c '
c=$0; T=40; E=""; O=vA$c
case $c in r21) O=r21;; d40) O=v22d40;; v23) O=v23;; early) T=55;; late) T=30;; b4) E="TS_BURST_READS=4";; sil) E="TS_EXTRA_ON_SILENT=1";; head) E="TS_PROPOSER_BURST_HEADROOM=0.08 TS_PROPOSER_BURST_MAX_V=20";; fb2) E="TS_FALLBACK_READS=2";; esac
[ -f out/$O/$1.json ] && exit 0
case $c in r21) OUT=r21 python3 sim21.py v21 $1;; d40) OUT=v22d40 TARGET=40 python3 sim22.py v22 $1;; *) env $E OUT=$O TARGET=$T python3 sim23.py v23 $1;; esac >> q_$c.err 2>&1'
echo QDONE
./backup.sh
