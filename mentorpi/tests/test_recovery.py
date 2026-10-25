"""Getting a stuck car going again (tests/run.sh test_recovery.py), explore_manager without driving: a goal
that fails where the car stands makes it back out and go on (the frontier is not written off); after
ESCAPE_MAX escapes that didn't help it stops and asks for help; Back out works while not exploring too."""
import sys, types, threading, time
sys.path.insert(0, '/home/ubuntu/web_ui')
import explore_manager as em

def explorer(escape_moves):
    e = em.Explorer.__new__(em.Explorer)
    e.log = types.SimpleNamespace(info=lambda *a: None, warn=print, error=print)
    e.state, e.goal, e.home, e.escapes, e.message = 'exploring', [1.0, 2.0, 0.0], [0, 0, 0], 0, ''
    e.recovering = e.backout_req = False
    e.failed_centers, e.blacklist = [], []
    e.sent, e.stopped = [], None
    e._send = lambda g: e.sent.append(g)
    e.stop = lambda reason='': setattr(e, 'stopped', reason) or setattr(e, 'state', 'stopped')
    def esc(force=False):
        if e.escapes >= em.ESCAPE_MAX and not force:
            return False
        e.escapes += 1
        return escape_moves
    e._escape = esc
    return e

e = explorer(True)
handled = e._stuck_recovery('stuck')
print(f'1. stuck, room to back out: handled {handled}, goal kept as target (not failed): {e.goal is None and not e.failed_centers}, escapes {e.escapes}')
assert handled and e.goal is None and e.escapes == 1 and e.stopped is None
e.state = 'returning'; e.goal = [0, 0, 0]
e._stuck_recovery('stuck')
print(f'2. stuck on the way home: backed out and sent home again: {e.sent == [[0, 0, 0]]}')
assert e.sent == [[0, 0, 0]]
e = explorer(True); e.escapes = em.ESCAPE_MAX
handled = e._stuck_recovery('stuck')
print(f'3. after {em.ESCAPE_MAX} escapes that did not help: stopped with "{e.stopped[:45]}..."')
assert handled and e.state == 'stopped' and e.stopped.startswith('stuck')
e = explorer(False)
handled = e._stuck_recovery('stuck')
print(f'4. no room to back out: left to the normal failure handling: {not handled}')
assert not handled
# 5. Back out while not exploring: runs the escape (forced), clears the flag afterwards
e = explorer(True); e.state = 'stopped'
e.slam = types.SimpleNamespace(running=lambda: True, pose=[0, 0, 0])
e.nav = types.SimpleNamespace(goal_active=lambda: False)
e.cmd_pub = types.SimpleNamespace(publish=lambda m: None)
e.active = lambda: False
e.escapes = em.ESCAPE_MAX
ok, msg = e.back_out()
for _ in range(50):
    if not e.recovering: break
    time.sleep(0.02)
print(f'5. Back out while stopped: {ok} "{msg}" -> "{e.message}", even after {em.ESCAPE_MAX} escapes')
assert ok and e.message == 'backed out' and not e.recovering
e.active = lambda: True
ok, msg = e.back_out()
print(f'6. Back out while exploring: asked for ({e.backout_req}), "{msg}"')
assert ok and e.backout_req
print('ALL OK')
