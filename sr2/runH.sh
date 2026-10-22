cd /workspace/sr2
( for L in 120 200; do for g in $(cat segs19.txt); do echo "$L $g"; done; done ) | xargs -P8 -L1 sh -c '
[ -f out/vHL$0/$1.json ] && exit 0
LIFT=$0 OUT=vHL$0 TARGET=40 python3 simH.py vH $1 >> qH_$0.err 2>&1'
echo HDONE
