#!/usr/bin/env python3
"""
Perceive-lock demo bus agent (CA-0015).

One small DroneEngage module over the de_databus python client, two modes:

  tap <log.jsonl> [listen_port]
      subscribe to the perception message types and append every received
      intermodule message as one JSON line: {"ts", "mt", "sd", "ms"}.

  invoke <ns> <act> <params_json> [result_timeout_s]
      send TYPE_AndruavMessage_CAPABILITY_INVOKE {id,ns,act,p} as an
      intermodule broadcast (de_comm routes it to the namespace owner) and
      print the matching CAPABILITY_RESULT line.

Env (both modes):
  DE_COMM_IP      module-bus target ip    (default 127.0.0.1)
  DE_COMM_PORT    module-bus target port  (default 61000)
  TAP_LISTEN_PORT this module's UDP port  (default: tap 61010, invoke 61011)

Nothing here is demo-hardware specific; the same script runs on x86 or the
Pis as long as the de_databus python package is importable (see _databus
path resolution below).
"""

import os
import sys
import time
import json
import signal
import random
import string

# de_databus python client: sibling repo in the drone_engage workspace, or
# explicit DE_DATABUS_PY env override.
_HERE = os.path.dirname(os.path.abspath(__file__))
for _cand in (
    os.environ.get("DE_DATABUS_PY", ""),
    os.path.join(_HERE, "..", "..", "..", "de_databus", "python"),
    os.path.join(_HERE, "..", "..", "de_databus", "python"),
):
    if _cand and os.path.isfile(os.path.join(_cand, "de_module.py")):
        sys.path.insert(0, _cand)
        break

from messages import (                              # noqa: E402
    TYPE_AndruavMessage_ID,
    TYPE_AndruavMessage_TrackingTarget_ACTION,
    TYPE_AndruavMessage_TrackingTargetLocation,
    TYPE_AndruavMessage_TrackingTarget_STATUS,
    TYPE_AndruavMessage_AI_Recognition_ACTION,
    TYPE_AndruavMessage_AI_Recognition_STATUS,
    TYPE_AndruavMessage_Sync_EventFire,
    TYPE_AndruavMessage_MODULE_CAPABILITIES,
    TYPE_AndruavMessage_CAPABILITY_INVOKE,
    TYPE_AndruavMessage_CAPABILITY_RESULT,
    TYPE_AndruavMessage_MODULE_STATE,
    ANDRUAV_PROTOCOL_MESSAGE_TYPE,
    ANDRUAV_PROTOCOL_MESSAGE_CMD,
    ANDRUAV_PROTOCOL_SENDER,
)
from de_module import CModule                       # noqa: E402
from de_facade_base import CFacade_Base             # noqa: E402

# 6546 = TYPE_AndruavMessage_AI_OBSERVATIONS in de_common messages.hpp;
# the python client has not been updated with the constant yet.
TYPE_AndruavMessage_AI_OBSERVATIONS = 6546


TAP_FILTER = [
    TYPE_AndruavMessage_ID,
    TYPE_AndruavMessage_TrackingTarget_ACTION,
    TYPE_AndruavMessage_TrackingTargetLocation,
    TYPE_AndruavMessage_TrackingTarget_STATUS,
    TYPE_AndruavMessage_AI_Recognition_ACTION,
    TYPE_AndruavMessage_AI_Recognition_STATUS,
    TYPE_AndruavMessage_Sync_EventFire,
    TYPE_AndruavMessage_MODULE_CAPABILITIES,
    TYPE_AndruavMessage_CAPABILITY_INVOKE,
    TYPE_AndruavMessage_CAPABILITY_RESULT,
    TYPE_AndruavMessage_MODULE_STATE,
    TYPE_AndruavMessage_AI_OBSERVATIONS,
]

INVOKE_FILTER = [
    TYPE_AndruavMessage_ID,
    TYPE_AndruavMessage_CAPABILITY_RESULT,
]

DEFAULT_UDP_DATABUS_PACKET_SIZE = 8192

cModule = CModule()
facade = CFacade_Base(cModule)
exit_me = False


def _module_id(prefix):
    return prefix + "".join(random.choice(string.digits) for _ in range(10))


def _quit(sig, frame):
    global exit_me
    exit_me = True


def _connect(listen_port):
    signal.signal(signal.SIGINT, _quit)
    signal.signal(signal.SIGTERM, _quit)
    cModule.init(
        os.environ.get("DE_COMM_IP", "127.0.0.1"),
        int(os.environ.get("DE_COMM_PORT", "61000")),
        "0.0.0.0",
        listen_port,
        DEFAULT_UDP_DATABUS_PACKET_SIZE,
    )


def cmd_tap(log_path, listen_port):
    log = open(log_path, "a", buffering=1)

    def on_message(message, length, jMsg):
        try:
            log.write(json.dumps({
                "ts": time.time(),
                "mt": jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_TYPE),
                "sd": jMsg.get(ANDRUAV_PROTOCOL_SENDER, ""),
                "ms": jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_CMD),
            }) + "\n")
        except Exception as e:
            print("tap log error:", e)

    cModule.m_OnReceive = on_message
    cModule.defineModule("gen", _module_id("tap"), "demotap01", "0.1", TAP_FILTER)
    _connect(listen_port)
    print(f"tap listening, logging to {log_path}")
    while not exit_me:
        time.sleep(0.2)
    cModule.uninit()


def cmd_invoke(ns, act, params, timeout_s, listen_port):
    # every module that owns a capability namespace answers a broadcast
    # invoke - non-owners reply err "unknown action". Collect all replies for
    # the id and prefer the ok:true one (the ns owner's).
    result = {"done": False, "id": _module_id("inv").replace("inv", "i")[:16],
              "resp": None, "resps": []}

    def on_message(message, length, jMsg):
        if jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_TYPE) != TYPE_AndruavMessage_CAPABILITY_RESULT:
            return
        cmd = jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_CMD) or {}
        if cmd.get("id") != result["id"]:
            return
        result["resps"].append(cmd)
        if cmd.get("ok"):
            result["resp"] = cmd
            result["done"] = True
        elif result["resp"] is None:
            result["resp"] = cmd

    cModule.m_OnReceive = on_message
    cModule.defineModule("gen", _module_id("op"), "demooper01", "0.1", INVOKE_FILTER)
    _connect(listen_port)

    # give the registration handshake a beat so de_comm knows us
    time.sleep(0.8)

    invoke = {"id": result["id"], "ns": ns, "act": act,
              "p": params, "dl": timeout_s}
    cModule.sendJMSG("", invoke, TYPE_AndruavMessage_CAPABILITY_INVOKE, True)

    deadline = time.time() + timeout_s + 2
    while not exit_me and not result["done"] and time.time() < deadline:
        time.sleep(0.05)

    cModule.uninit()
    out = result["resp"] if result["resp"] is not None \
        else {"id": result["id"], "ok": False, "err": "no result received"}
    if len(result["resps"]) > 1:
        print(json.dumps({"all": result["resps"]}))
    print(json.dumps(out))
    sys.exit(0 if result["done"] else 1)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)

    mode = sys.argv[1]
    if mode == "tap":
        log_path = sys.argv[2] if len(sys.argv) > 2 else "bus.jsonl"
        listen = int(sys.argv[3]) if len(sys.argv) > 3 else int(os.environ.get("TAP_LISTEN_PORT", "61010"))
        cmd_tap(log_path, listen)
    elif mode == "invoke":
        if len(sys.argv) < 5:
            print("usage: bus_agent.py invoke <ns> <act> <params_json> [timeout_s]")
            sys.exit(2)
        ns, act = sys.argv[2], sys.argv[3]
        params = json.loads(sys.argv[4]) if sys.argv[4] else {}
        timeout_s = float(sys.argv[5]) if len(sys.argv) > 5 else 5.0
        listen = int(os.environ.get("INVOKE_LISTEN_PORT", "61011"))
        cmd_invoke(ns, act, params, timeout_s, listen)
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == "__main__":
    main()
