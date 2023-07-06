import json
import sys
from time import time_ns

from mcap.writer import Writer

import pandas as pd
import json

file = "./result.mcap"

csv = "/home/naoto/Desktop/mouse/Banshee/tools/param_tuner/logs/latest.csv"

csv_df = pd.read_csv(csv)
csv_df.to_json("./tmp.json", orient="records")

col = csv_df.columns
with open("./tmp.json") as f:
    d = json.load(f)
    print(d)
with open(file, "wb") as stream:
    writer = Writer(stream)
    writer.start()
    schema_id = writer.register_schema(
        name="sample",
        encoding="jsonschema",
        data=json.dumps({
            "type": "object",
            "properties": {
                "index": {"type": "number"},
                "ideal_v": {"type": "number"},
                "v_c": {"type": "number"},
                "v_c2": {"type": "number"},
                "v_l": {"type": "number"},
                "v_r": {"type": "number"},
                "accl": {"type": "number"},
                "accl_x": {"type": "number"},
                "ideal_w": {"type": "number"},
                "w_lp": {"type": "number"},
                "alpha": {"type": "number"},
                "ideal_dist": {"type": "number"},
                "dist": {"type": "number"},
                "ideal_ang": {"type": "number"},
                "ang": {"type": "number"},
                "left90": {"type": "number"},
                "left45": {"type": "number"},
                "front": {"type": "number"},
                "right45": {"type": "number"},
                "right90": {"type": "number"},
                "left90_d": {"type": "number"},
                "left45_d": {"type": "number"},
                "front_d": {"type": "number"},
                "right45_d": {"type": "number"},
                "right90_d": {"type": "number"},
                "left90_far_d": {"type": "number"},
                "front_far_d": {"type": "number"},
                "right90_far_d": {"type": "number"},
                "battery": {"type": "number"},
                "duty_l": {"type": "number"},
                "duty_r": {"type": "number"},
                "motion_state": {"type": "number"},
                "duty_sen": {"type": "number"},
                "dist_mod90": {"type": "number"},
                "sen_dist_l45": {"type": "number"},
                "sen_dist_r45": {"type": "number"},
                "timestamp": {"type": "number"},
            },
        }).encode(),
    )

    channel_id = writer.register_channel(
        schema_id=schema_id,
        topic="/log",
        message_encoding="json",
    )

    with open("./tmp.json") as f:
        d = json.load(f)
        c = 0
        for j in d:
            c += 1
            writer.add_message(
                channel_id=channel_id,
                log_time=c * 1000000,
                data=json.dumps(j).encode("utf-8"),
                publish_time=c * 1000000,
            )

    writer.finish()
