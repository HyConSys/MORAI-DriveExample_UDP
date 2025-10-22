"""
pfaces_symcontrol_client.py

Python port of pFacesSymControlClient (original C++ implementation).
Based on the uploaded C++ source. :contentReference[oaicite:1]{index=1}
"""

from typing import List, Tuple
import requests
import time

# Constants mirroring the C++ defines
ONLINE_MODE_DICT_KEY_mode = "mode"  # modes: busy, collect_synth, distribute_control
ONLINE_MODE_DICT_KEY_is_synth_requested = "is_synth_requested"
ONLINE_MODE_DICT_KEY_obst_set = "obst_set"
ONLINE_MODE_DICT_KEY_target_set = "target_set"
ONLINE_MODE_DICT_KEY_is_last_synth_request = "is_last_synth_request"
ONLINE_MODE_DICT_KEY_is_control_requested = "is_control_requested"
ONLINE_MODE_DICT_KEY_current_state = "current_state"
ONLINE_MODE_DICT_KEY_is_last_control_request = "is_last_control_request"
ONLINE_MODE_DICT_KEY_is_control_ready = "is_control_ready"
ONLINE_MODE_DICT_KEY_is_control_recieved = "is_control_recieved"
ONLINE_MODE_DICT_KEY_actions_list = "actions_list"
ONLINE_MODE_TRUE_STRING = "true"
ONLINE_MODE_FALSE_STRING = "false"


# --- small internal helpers (port of internal:: functions) ---
def str_split(src: str, delimiter: str, keep_null: bool = False) -> List[str]:
    if delimiter == "":
        return [src]
    parts = []
    start = 0
    dlen = len(delimiter)
    while True:
        idx = src.find(delimiter, start)
        if idx == -1:
            token = src[start:]
            if keep_null or token != "":
                parts.append(token)
            break
        token = src[start:idx]
        if keep_null or token != "":
            parts.append(token)
        start = idx + dlen
    return parts


def str_replace_all(s: str, old: str, new: str) -> str:
    return s.replace(old, new)


def sstr_to_float_vector(s: str, separator: str = ',') -> List[float]:
    if s == "":
        return []
    parts = s.split(separator)
    out = []
    for p in parts:
        p = p.strip()
        if p == "":
            continue
        try:
            out.append(float(p))
        except ValueError:
            # mimic the C++ behaviour which would push a default-constructed float (0.0)
            out.append(0.0)
    return out


# --- main class ---
class pFacesSymControlClient:
    """
    Synchronous Python port of the C++ pFacesSymControlClient.
    Usage:
      client = pFacesSymControlClient(base_uri, token_path)
      client.send_synthesis_request([...], "(1,2)", is_last_synth_request=False)
      actions = client.get_control_actions([0.1, 2.3], is_last_control_request=False)
    """

    def __init__(self, base_uri: str, token: str, poll_interval: float = 0.1):
        """
        :param base_uri: base URL of the pFaces server, e.g. "http://localhost:5000"
        :param token: path or token appended to base_uri used by the C++ client (client.request(method, token))
                      Typical full request URL will be: base_uri.rstrip('/') + '/' + token.lstrip('/')
        :param poll_interval: seconds to sleep between polling checks (default 0.1s)
        """
        self.base_uri = base_uri.rstrip('/')
        self.token = token.lstrip('/')
        self.url = f"{self.base_uri}/{self.token}"
        self.session = requests.Session()
        self.poll_interval = poll_interval

    def make_request(self, method: str, payload):
        """Synchronous request wrapper. method is 'GET' or 'PUT' (or 'POST' if needed)."""
        try:
            if method.upper() == 'GET':
                r = self.session.get(self.url, timeout=10)
            elif method.upper() == 'PUT':
                r = self.session.put(self.url, json=payload, timeout=10)
            elif method.upper() == 'POST':
                r = self.session.post(self.url, json=payload, timeout=10)
            else:
                raise ValueError(f"Unsupported HTTP method: {method}")
            r.raise_for_status()
            # If there's JSON return it; otherwise return empty dict
            if r.text:
                try:
                    return r.json()
                except ValueError:
                    return {}
            return {}
        except requests.RequestException as e:
            # Mirror C++ which printed the exception and returned empty json
            print(f"make_request: HTTP error: {e}")
            return {}

    def pfaces_put_values(self, keyvals: List[Tuple[str, str]]):
        """
        Send a PUT with an object mapping keys -> string values.
        Mirrors the C++ put (json::value::object filled with strings).
        """
        put_payload = {}
        for k, v in keyvals:
            put_payload[k] = v
        # send synchronously; ignore return value
        self.make_request('PUT', put_payload)

    def pfaces_get_values(self, keys: List[str]) -> List[str]:
        """
        Retrieve the current dictionary on server via GET and return values for requested keys
        in the same order as keys argument. Missing keys are returned as empty string.
        """
        response = self.make_request('GET', []) or {}
        # response expected to be dict-like
        ret = ["" for _ in keys]
        for i, key in enumerate(keys):
            ret[i] = response[key]
        return ret

    # ---------------- high-level API ----------------
    def send_synthesis_request(self, obstacles_intervals: List[str], target_interval: str,
                               is_last_synth_request: bool):
        """
        Mirrors C++ sendSynthesisRequest:
          - waits until mode == "collect_synth"
          - sets target_set, obst_set, is_last_synth_request, is_synth_requested=true via PUT
        """
        obsts = "|".join(obstacles_intervals)
        #print(f"sendSynthesisRequest:: Sending synthesis request to pFaces server with obst: {obsts} and target: {target_interval}")

        # wait until server mode becomes collect_synth
        while True:
            mode = self.pfaces_get_values([ONLINE_MODE_DICT_KEY_mode])[0]
            if mode == "collect_synth":
                break
            time.sleep(self.poll_interval)

        keyvals = []
        keyvals.append((ONLINE_MODE_DICT_KEY_target_set, target_interval))
        keyvals.append((ONLINE_MODE_DICT_KEY_obst_set, obsts))
        keyvals.append((ONLINE_MODE_DICT_KEY_is_last_synth_request,
                        ONLINE_MODE_TRUE_STRING if is_last_synth_request else ONLINE_MODE_FALSE_STRING))
        keyvals.append((ONLINE_MODE_DICT_KEY_is_synth_requested, ONLINE_MODE_TRUE_STRING))

        self.pfaces_put_values(keyvals)
        #print("sendSynthesisRequest:: Done.")

    def get_control_actions(self, current_state: List[float], is_last_control_request: bool) -> List[List[float]]:
        """
        Mirrors C++ getControlActions:
          - format current_state as "(a,b,c)"
          - wait until mode == "distribute_control"
          - PUT current_state, is_last_control_request, is_control_requested=true
          - wait until is_control_ready == "true"
          - GET actions_list and parse it into list of float vectors
          - acknowledge by setting is_control_recieved=true
        """
        # format state as "(1.0,2.0,3.0)"
        state = "(" + ",".join(str(x) for x in current_state) + ")"

        # wait until server mode becomes distribute_control
        while True:
            mode = self.pfaces_get_values([ONLINE_MODE_DICT_KEY_mode])[0]
            if mode == "distribute_control":
                break
            time.sleep(self.poll_interval)

        keyvals = []
        keyvals.append((ONLINE_MODE_DICT_KEY_current_state, state))
        keyvals.append((ONLINE_MODE_DICT_KEY_is_last_control_request,
                        ONLINE_MODE_TRUE_STRING if is_last_control_request else ONLINE_MODE_FALSE_STRING))
        keyvals.append((ONLINE_MODE_DICT_KEY_is_control_requested, ONLINE_MODE_TRUE_STRING))

        self.pfaces_put_values(keyvals)

        # wait until server signals control is ready
        while True:
            is_ready = self.pfaces_get_values([ONLINE_MODE_DICT_KEY_is_control_ready])[0]
            if is_ready == ONLINE_MODE_TRUE_STRING:
                break
            time.sleep(self.poll_interval)

        str_all_actions = self.pfaces_get_values([ONLINE_MODE_DICT_KEY_actions_list])[0]

        ret: List[List[float]] = []
        if str_all_actions == "empty-actions":
            # nothing to do
            pass
        else:
            vec_actions = str_split(str_all_actions, "|")
            for a in vec_actions:
                a = str_replace_all(a, "(", "")
                a = str_replace_all(a, ")", "")
                floats = sstr_to_float_vector(a, ',')
                ret.append(floats)

        # acknowledge receiving the actions
        self.pfaces_put_values([(ONLINE_MODE_DICT_KEY_is_control_recieved, ONLINE_MODE_TRUE_STRING)])

        return ret

