import json
import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import analyze_exchange


class AnalyzeExchangeTest(unittest.TestCase):
    def test_matches_sequences_and_latency(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.jsonl"
            received = root / "received.jsonl"
            source.write_text(
                "\n".join([
                    json.dumps({
                        "_transport": {
                            "seq": "vehicle-00000000",
                            "transport_sent_unix_ns": 1_000_000_000,
                        }
                    }),
                    json.dumps({
                        "_transport": {
                            "seq": "vehicle-00000001",
                            "transport_sent_unix_ns": 2_000_000_000,
                        }
                    }),
                ]) + "\n",
                encoding="utf-8",
            )
            inner = {
                "_transport": {
                    "seq": "vehicle-00000000",
                    "transport_sent_unix_ns": 1_000_000_000,
                }
            }
            received.write_text(
                json.dumps({
                    "t": 1.050,
                    "payload": {"data": json.dumps(inner)},
                }) + "\n",
                encoding="utf-8",
            )
            result = analyze_exchange.analyze(
                "test", source, received)
            self.assertEqual(result["sent"], 2)
            self.assertEqual(result["matched"], 1)
            self.assertEqual(result["pdr"], 0.5)
            self.assertAlmostEqual(
                result["latency_clock_dependent_ms"]["p50"], 50.0)


if __name__ == "__main__":
    unittest.main()
