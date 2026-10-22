cd /workspace/sr2
while ! grep -q SWODONE sweepo.out; do sleep 20; done
cat segs_other.txt | xargs -P8 -I{} sh -c 'OUT=v23lowv TARGET=40 python3 sim23.py v23 {} >> swo_v23.err 2>&1'
echo SWO2DONE
