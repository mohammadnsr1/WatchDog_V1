"""Token usage is counted once per API message, not once per transcript record."""
import json

from analysis.transcripts import read_transcript


def _rec(mid, out, block):
    return {"type": "assistant", "timestamp": "2026-10-20T00:00:00Z",
            "message": {"id": mid, "model": "claude-sonnet-5", "content": [block],
                        "usage": {"input_tokens": 3, "output_tokens": out,
                                  "cache_read_input_tokens": 100, "cache_creation_input_tokens": 10}}}


def test_usage_repeated_across_content_blocks_counts_once(tmp_path):
    p = tmp_path / "t.jsonl"
    recs = [
        # one API response split into three records (thinking, text, tool_use), same usage on each
        _rec("msg_a", 423, {"type": "thinking", "thinking": "..."}),
        _rec("msg_a", 423, {"type": "text", "text": "..."}),
        _rec("msg_a", 423, {"type": "tool_use", "name": "mcp__robot-backend-watchdog__look"}),
        # a second response, one record
        _rec("msg_b", 104, {"type": "text", "text": "..."}),
    ]
    p.write_text("\n".join(json.dumps(r) for r in recs) + "\n", encoding="utf-8")
    t = read_transcript(p)
    assert t.n_assistant == 2
    assert t.tokens_out == 423 + 104
    assert t.tokens_in == 6
    assert t.cache_read_tokens == 200
    assert t.cache_creation_tokens == 20
