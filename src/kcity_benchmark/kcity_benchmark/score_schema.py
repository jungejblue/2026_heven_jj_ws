"""Serializable detailed score view, including the computed quality score."""


def score_details(card, extra):
    names = ("completion", "safety", "mission", "efficiency",
             "comfort", "quality", "total")
    scores = {
        name: None if getattr(card, name) is None else round(getattr(card, name), 3)
        for name in names
    }
    success = extra["mission_success_rate_pct"]
    scores["mission_success_rate_pct"] = None if success is None else round(success, 3)
    scores["active_quality_weights"] = {
        name: round(weight, 4)
        for name, weight in card.active_quality_weights.items()
    }
    scores["safety_details"] = {
        name: round(value, 4)
        for name, value in extra["safety_details"].items()
    }
    return scores
