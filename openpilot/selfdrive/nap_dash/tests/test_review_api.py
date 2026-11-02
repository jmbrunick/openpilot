import json

from openpilot.selfdrive.nap_dash.review_api import read_flags


def test_lists_newest_first_and_kinds(tmp_path):
  (tmp_path / "review_flags.jsonl").write_text(
    "\n".join(json.dumps({"kind": "both", "reason": f"both_maps_50_to_{55 + i * 5}", "mph": 55 + i * 5, "wall": 1.0 + i,
                          "route": "r", "segment": i, "secret": "x"}) for i in range(3)) + "\nnot json\n")
  (tmp_path / "review_single.jsonl").write_text(json.dumps({"kind": "single", "reason": "tesla_only_30_to_55"}) + "\n")
  d = read_flags(str(tmp_path))
  assert [f["mph"] for f in d["flags"]] == [65, 60, 55]
  assert "secret" not in d["flags"][0] and d["flags"][0]["segment"] == 2
  assert read_flags(str(tmp_path), kind="single")["flags"][0]["reason"] == "tesla_only_30_to_55"
  assert read_flags(str(tmp_path / "none"))["flags"] == []
  assert len(read_flags(str(tmp_path), limit=1)["flags"]) == 1
