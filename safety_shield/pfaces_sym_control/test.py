from .client import pFacesSymControlClient


def main():
    client = pFacesSymControlClient("http://127.0.0.1:12345", "pFaces/REST/dictionary/morai_acas")

    # simple test to get data
    values = client.pfaces_get_values(["mode", "is_synth_requested", "is_control_recieved"])
    print("Values:", values)


    # assert deffault values, if fails and the values are not empty, then pFAces-Symbolic control was used before
    # if so, please reset the server
    assert values == ["collect_synth", "false", "false"], "Unexpected initial values, if not empty, please reset the pFaces-Symbolic Control server"


    # send simple synthesis request
    is_last_synth_request = True
    obstacles_intervals = ["0.9,1.3,0.0,8.6,-3.4,3.4", 
                           "2.1,2.5,0.0,4.9,-3.4,3.4",
                           "2.1,2.5,5.9,10.0,-3.4,3.4",
                           "3.3,3.7,0.0,8.6,-3.4,3.4",
                           "4.5,4.9,1.1,10.0,-3.4,3.4",
                           "5.7,6.1,0.0,5.9,-3.4,3.4",
                           "5.7,6.1,6.9,10.0,-3.4,3.4",
                           "6.9,7.3,1.1,10.0,-3.4,3.4",
                           "8.1,8.5,0.0,8.5,-3.4,3.4",
                           "8.3,9.1,8.3,8.5,-3.4,3.4",
                           "9.1,10.0,7.1,7.3,-3.4,3.4",
                           "8.3,9.1,5.9,6.1,-3.4,3.4",
                           "9.1,10.0,4.7,4.9,-3.4,3.4",
                           "8.3,9.1,3.5,3.7,-3.4,3.4",
                           "9.1,10.0,2.3,2.5,-3.4,3.4"]
    target = "8.6,9.9,0.2,2.2,-3.4,3.4"
    client.send_synthesis_request(obstacles_intervals, target, is_last_synth_request)


    # get control actions for a given state
    is_last_control_request = True
    current_state = [0.5, 1.5, 1.5]
    actions = client.get_control_actions(current_state, is_last_control_request)
    print("Actions:", actions)


if __name__ == '__main__':
    main()
