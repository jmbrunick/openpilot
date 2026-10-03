"""Map road-class mutation definitions (see tesla_preap_longitudinal_mutations.py).

A golf-cart `highway=path maxspeed=5 mph` must never be matched as the posted
limit (Oct 1-3 routes locked HUD MAX to 5 mph). Kept in its own part so the
parent defs stay untouched.
"""
from tesla_preap_mutation_common import (
  HistoricalMutation,
  ROAD_CLASS_TEST_PATH,
)


MUTATIONS_C = (
  HistoricalMutation(
    name="road-class-filter-removed",
    source_path="selfdrive/mapd/osm_db.py",
    original=(
      b"      highway = row[\"highway\"] or \"\"\n" +
      b"      if _is_non_road_highway(highway):\n" +
      b"        continue\n"
    ),
    replacement=b"      highway = row[\"highway\"] or \"\"\n",
    test_nodes=(
      f"{ROAD_CLASS_TEST_PATH}::test_non_road_way_alone_is_not_matched",
      f"{ROAD_CLASS_TEST_PATH}::test_non_road_way_does_not_beat_a_farther_tagged_road",
    ),
  ),
  HistoricalMutation(
    name="road-class-filter-missing-from-next-limit",
    source_path="selfdrive/mapd/osm_db.py",
    original=b"      if _is_non_road_highway(highway):\n        continue\n",
    replacement=b"      if along_route is None and _is_non_road_highway(highway):\n        continue\n",
    test_nodes=(
      f"{ROAD_CLASS_TEST_PATH}::test_next_limit_ignores_a_non_road_way_ahead",
    ),
  ),
  HistoricalMutation(
    name="road-class-filter-moved-into-candidates",
    source_path="selfdrive/mapd/osm_db.py",
    original=b"    return list(self._con.execute(q, ids))\n\n  def _best_match(",
    replacement=(
      b"    return [r for r in self._con.execute(q, ids) if not _is_non_road_highway(r[\"highway\"])]\n" +
      b"\n  def _best_match("
    ),
    test_nodes=(
      f"{ROAD_CLASS_TEST_PATH}::test_junction_tagged_ring_is_found_even_in_a_non_road_class",
    ),
  ),
)
