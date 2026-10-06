"""Multi-node singular-region regression; offline waypoint retry harness."""
import copy
import numpy as np
from adapters.ik_selection import pack_model, pose_error


def verify_sequences(selector, profile):
    model = pack_model(profile)
    base = [.3, -1, .9, -.7, .035, .2]
    state = dict(q_rad=base.copy(), step_index=0)
    nodes = []
    events = []

    def attempt(sequence, phase, goal, expected=0):
        nonlocal state
        before = copy.deepcopy(state)
        target = selector._fk(model, goal)
        result = selector.select(profile, target.tolist(), state)
        valid = result['code'] == 0
        input_unchanged = state == before
        if valid:
            state = copy.deepcopy(result['next_state'])
            actual = selector._fk(model, state['q_rad'])
            pe, re = pose_error(actual, target)
            sigma = selector.sigma_min(model, state['q_rad'])
        else:
            pe = re = sigma = None
        advanced = state['step_index'] == before['step_index'] + 1
        retained = selector._fk(model, state['q_rad'])
        retained_pe, retained_re = pose_error(retained, target)
        previous_sigma = selector.sigma_min(model, before['q_rad'])
        cap = np.minimum(selector.config.max_step_rad,
                         np.array(profile['qd_max_rad_s'])*selector.config.dt_s)
        if valid and min(previous_sigma, sigma)<selector.config.sigma_soft:
            cap *= selector.config.near_step_scale
        motion_valid = True
        if valid:
            q = np.array(state['q_rad'])
            motion_valid = (np.all(np.isfinite(q))
                            and np.all(np.abs(q-np.array(before['q_rad']))<=cap+1e-12)
                            and np.all(q>=np.array(profile['q_min_rad'])+selector.config.joint_margin_rad)
                            and np.all(q<=np.array(profile['q_max_rad'])-selector.config.joint_margin_rad)
                            and (result['data']['mode']=='singular_hold' or sigma>selector.config.sigma_hard))
        ok = (result['code'] == expected and input_unchanged
              and (advanced if valid else state == before)
              and motion_valid
              and (pe <= selector.config.position_tol_m and re <= selector.config.orientation_tol_rad
                   if valid else result['data'] is None and result['next_state'] is None))
        node = dict(sequence=sequence, node_index=len(nodes), phase=phase,
                    target_T_base_tool=target.tolist(), previous_state=before,
                    retained_state=copy.deepcopy(state), code=result['code'], expected_code=expected,
                    passed=bool(ok), state_advanced=advanced,
                    step_cap_rad=cap.tolist(), previous_sigma_min=previous_sigma,
                    q_changed=state['q_rad'] != before['q_rad'],
                    position_error_m=pe, orientation_error_rad=re, sigma_min=sigma,
                    retained_state_position_error_m=retained_pe,
                    retained_state_orientation_error_rad=retained_re,
                    retained_state_sigma_min=selector.sigma_min(model, state['q_rad']),
                    mode=(result['data'] or {}).get('mode'), details=result['details'])
        # Candidate arrays are already exercised by the single-node suite.
        node['details'] = {k: v for k, v in node['details'].items() if k != 'candidates'}
        nodes.append(node)
        return valid

    for wrist in (.03, .025, .02, .015, .01, .005, 0., 0.):
        goal = base.copy(); goal[4] = wrist
        attempt('approach_hold_depart', 'hold' if wrist == 0. else 'approach', goal)
    for wrist in (.003, .008, .013, .018, .023, .028, .033):
        goal = base.copy(); goal[4] = wrist
        attempt('approach_hold_depart', 'depart', goal)

    # A failed node must stop the active segment. The following old segment
    # endpoint is deliberately discarded, then new nodes start from retained q.
    goal = base.copy(); goal[4] = .2
    stopped_state = copy.deepcopy(state)
    failure_index = len(nodes)
    failed = not attempt('stop_and_replan', 'step_unreachable', goal, 2002)
    events.append(dict(event='stop_segment', failed_node=failure_index,
                       state=copy.deepcopy(state), abandoned_node_count=1,
                       abandoned_target_q_rad=[.3, -1, .9, -.7, .25, .2]))
    start = np.array(state['q_rad']); endpoint = np.array(goal)
    count = int(np.ceil(np.max(np.abs(endpoint-start)) / .005))
    events.append(dict(event='replan_from_retained_state', state=copy.deepcopy(state),
                       method='offline_joint_waypoint_subdivision', node_count=count,
                       maximum_waypoint_delta_rad=.005))
    for index in range(1, count+1):
        attempt('stop_and_replan', 'replacement_node', (start+(endpoint-start)*index/count).tolist())

    # Exact singular state holding and leaving is a separate consecutive path.
    state = dict(q_rad=[.3, -1, .9, -.7, 0., .2], step_index=0)
    for wrist in (0., 0., .003, .008, .013):
        goal = base.copy(); goal[4] = wrist
        attempt('exact_hold_and_exit', 'hold' if wrist == 0. else 'depart', goal)

    checks = [
        dict(name='sequence_all_node_contracts', passed=all(n['passed'] for n in nodes)),
        dict(name='sequence_dls_then_hold_then_exit', passed=(
            any(n['mode']=='dls_near_singular' and n['phase']=='hold' for n in nodes)
            and any(n['phase']=='hold' and not n['q_changed'] for n in nodes)
            and all(n['code']==0 for n in nodes if n['phase']=='depart'))),
        dict(name='sequence_failed_segment_stops_state', passed=(failed
             and nodes[failure_index]['retained_state']==stopped_state
             and not nodes[failure_index]['state_advanced'])),
        dict(name='sequence_replan_reaches_original_goal', passed=False),
    ]
    # Last comparison uses the actual endpoint of the replacement segment,
    # rather than the later exact-hold sequence's goal.
    endpoint_node = [n for n in nodes if n['phase']=='replacement_node'][-1]
    checks[-1]['passed'] = (all(n['code']==0 for n in nodes if n['phase']=='replacement_node')
                           and endpoint_node['position_error_m'] is not None
                           and endpoint_node['position_error_m'] <= selector.config.position_tol_m
                           and endpoint_node['orientation_error_rad'] <= selector.config.orientation_tol_rad)
    return dict(nodes=nodes, events=events, checks=checks,
                scope='offline test harness; no collision, acceleration or full Cartesian replanner'), checks
