cd /workspace/sr2
while ! grep -q RSDONE rescore.out; do sleep 30; done
( for c in early late b4 sil head fb2; do for g in $(cat segs_all.txt); do echo "$c $g"; done; done ) | xargs -P6 -L1 sh -c '
c=$0; T=40; E=""
case $c in early) T=55;; late) T=30;; b4) E="TS_BURST_READS=4";; sil) E="TS_EXTRA_ON_SILENT=1";; head) E="TS_PROPOSER_BURST_HEADROOM=0.08 TS_PROPOSER_BURST_MAX_V=20";; fb2) E="TS_FALLBACK_READS=2";; esac
env $E OUT=vA$c TARGET=$T python3 sim23.py v23 $1 >> swA_$c.err 2>&1'
echo SWADONE
./backup.sh
