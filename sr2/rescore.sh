cd /workspace/sr2
cat segs.txt segs_other.txt > segs_all.txt
( for c in r354 r21 d40 v23; do for g in $(cat segs_all.txt); do echo "$c $g"; done; done ) | xargs -P6 -L1 sh -c 'c=$0; case $c in r354) OUT=r354 python3 sim.py 354 $1;; r21) OUT=r21 python3 sim21.py v21 $1;; d40) OUT=v22d40 TARGET=40 python3 sim22.py v22 $1;; v23) OUT=v23 TARGET=40 python3 sim23.py v23 $1;; esac >> rs_$c.err 2>&1'
echo RSDONE
./backup.sh
