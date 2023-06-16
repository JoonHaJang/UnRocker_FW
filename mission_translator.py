import json

import json

def json_to_txt(json_file, txt_file):
    with open(json_file, 'r') as f:
        data = json.load(f)

    mission_items = data["mission"]["items"]

    with open(txt_file, 'w') as f:
        for i, item in enumerate(mission_items):
            params = item["params"]
            line = f"{i} {item['command']} {item['frame']} {item['doJumpId']} {params[0]} {params[1]} {params[2]} {params[3]} {params[4]} {params[5]} {params[6]} {item['AltitudeMode']}"
            f.write(line)

# Replace 'input.json' with your JSON file name and 'output.txt' with the desired output text file name.
json_to_txt('mission.json', 'mission2.txt')
