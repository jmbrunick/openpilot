cd /workspace/sr2
for m in v2 354; do cat segs.txt | xargs -P4 -I{} sh -c "[ -f out/$m/{}.json ] || python3 sim.py $m {} >> sim_$m.out 2>&1"; done
echo ALLDONE >> sim_v2.out
