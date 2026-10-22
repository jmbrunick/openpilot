cd /workspace/sr2
while ! grep -q SW23DONE sweep23.out || ! grep -q DLDONE dlother.out; do sleep 20; done
( for c in r354 r21 d40; do for g in $(cat segs_other.txt); do echo "$c $g"; done; done ) | xargs -P8 -L1 sh -c 'c=$0; case $c in r354) OUT=r354 python3 sim.py 354 $1;; r21) OUT=r21 python3 sim21.py v21 $1;; d40) OUT=v22d40 TARGET=40 python3 sim22.py v22 $1;; esac >> swo_$c.err 2>&1'
echo SWODONE
