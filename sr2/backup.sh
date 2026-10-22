#!/bin/bash
# push sr2 scripts + scored results to GitHub (orphan branch) as a wipe backup
set -e
B=/workspace/sr2_bk; mkdir -p $B; cd $B
[ -d .git ] || { git init -q; git remote add origin https://github.com/jmbrunick/openpilot.git; git checkout -q --orphan cursor/sr2-replay-harness; }
rm -rf sr2; mkdir sr2; (cd /workspace/sr2 && tar cf - --exclude=ctx --exclude=ql --exclude="pkg*" --exclude=f7 --exclude=__pycache__ --exclude=out .) | tar xf - -C sr2; [ -d /workspace/sr2/out ] && tar czf sr2/out.tgz -C /workspace/sr2 out || true
mkdir -p results; cp /workspace/sr2/*.txt results/ 2>/dev/null || true
git add -A; git -c user.name=jmbrunick -c user.email=157572262+jmbrunick@users.noreply.github.com commit -qm "sr2 backup $(date +%F_%T)" || true
git push -q -f origin cursor/sr2-replay-harness
