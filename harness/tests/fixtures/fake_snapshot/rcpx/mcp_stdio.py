# SPDX-License-Identifier: AGPL-3.0-only
"""A stand-in for the measured bridge (rcpx.mcp_stdio), for the sim server's tests: two tools (look, which
names one picture, and claim, which ends the episode as the episode server does) and one resource. No socket and
no simulator."""
import json
import os

import mcp.types as types

CLAIM = "claim"
LISTING = {"tools": [{"name": "look", "description": "Take one picture."}], "about": "fixture conventions",
           "resources": [{"uri": "robot://task", "description": "the task"}]}


def instructions_for(listing, practices):
    return f"fixture instructions: {len((listing or {}).get('tools', []))} tool(s); practices: {practices}"


class Bridge:
    def __init__(self, ep_dir, timeout_s=900.0):
        self.ep_dir, self.timeout_s, self.listing, self.looks = ep_dir, timeout_s, None, 0

    def start(self):
        with open(os.path.join(self.ep_dir, "episode.json"), encoding="utf-8") as f:
            json.load(f)
        self.listing = LISTING
        return "fixture practices"

    async def list_tools(self):
        schema = {"type": "object", "properties": {}}
        return types.ListToolsResult(tools=[types.Tool(name="look", description="Take one picture.",
                                                       input_schema=schema),
                                            types.Tool(name=CLAIM, description="End the episode.",
                                                       input_schema=schema)])

    async def call_tool(self, name, arguments):
        if name == CLAIM:
            with open(os.path.join(self.ep_dir, "episode_end.json"), "w", encoding="utf-8") as f:
                json.dump({"reason": "claim", "args": arguments}, f)
            doc = {"claimed": True, "args": arguments}
        elif name == "look":
            self.looks += 1
            frame = f"frames/look{self.looks:03d}.png"
            with open(os.path.join(self.ep_dir, frame), "wb") as f:
                f.write(b"fixture picture %d" % self.looks)
            doc = {"ok": True, "frames": [frame]}
        else:
            doc = {"refused": {"check": "unknown_tool", "why": f"no tool {name}"}}
        return types.CallToolResult(content=[types.TextContent(text=json.dumps(doc))], is_error="refused" in doc)

    async def list_resources(self):
        return types.ListResourcesResult(resources=[types.Resource(uri="robot://task", name="task")])

    async def list_resource_templates(self):
        return types.ListResourceTemplatesResult(resource_templates=[])

    async def read_resource(self, uri):
        return types.ReadResourceResult(contents=[types.TextResourceContents(uri=uri, text="fixture task")])
