"""Narrator: sections, facts, templates, the provider interface, and the Claude
provider with a fake client."""
from types import SimpleNamespace

from security_task_manager.narrator import (ClaudeProvider, LLMProvider, Narrator, facts_for,
                                            make_provider, template_sentence)
from security_task_manager.zones import ZoneMap

# police_map2 sections (security_bringup/config/narrator.yaml)
POLYS = [-1.76, -2.71, -0.58, -1.33, -3.78, 1.39, -4.95, 0.01,
         -0.58, -1.33, 0.59, 0.05, -2.60, 2.77, -3.78, 1.39,
         0.59, 0.05, 1.77, 1.43, -1.42, 4.15, -2.60, 2.77]
ZONES = ZoneMap.from_flat(['A', 'B', 'C'], ['dock end', 'middle', 'far end'], POLYS)


def row(event, marker=42, x=0.21, y=2.30, level=1, robot=(-1.14, 0.56)):
    return {'event': event, 'marker_id': marker, 'warning_level': level, 'x_map': x,
            'y_map': y, 'robot_x': robot[0], 'robot_y': robot[1], 'patrol_pass': 2}


def test_sections_of_dock_waypoints_and_object():
    assert ZONES.zone_at(-2.74, -0.05).name == 'A'      # dock
    assert ZONES.zone_at(-1.14, 0.56).name == 'B'       # WP2
    assert ZONES.zone_at(0.21, 2.30).name == 'C'        # object 42 draft place
    assert ZONES.zone_at(1.0, 0.0).name in 'BC'         # just outside: nearest section
    assert ZONES.zone_at(30.0, 30.0) is None            # far away: no section


def test_templates_read_like_the_examples():
    assert template_sentence(facts_for(row('detected'), ZONES)) == \
        'In section C, a suspicious object (#42) has been detected.'
    assert template_sentence(facts_for(row('seen_again', level=2), ZONES)) == \
        'In section C, object #42 is still there and has not moved.'
    # follow_start has no object position: the robot's section is used
    assert template_sentence(facts_for(row('follow_start', x='', y='', level=3), ZONES)) == \
        'In section B, someone just picked up object #42 and I am following it.'


def test_robot_events_use_the_robot_position_and_noise_is_skipped():
    f = facts_for(row('go_charge', marker='', robot=(-2.5, 0.0)), ZONES)
    assert template_sentence(f) == 'In section A, I am heading back to my dock.'
    assert facts_for(row('moving_update'), ZONES) is None
    assert facts_for(row('nav_retry'), ZONES) is None


def test_without_a_provider_the_template_is_used():
    n = Narrator(make_provider('none'))
    text, source = n.narrate(facts_for(row('detected'), ZONES))
    assert source == 'template' and text.startswith('In section C,')
    try:
        make_provider('unknown-llm')
        assert False, 'unknown provider must be rejected'
    except ValueError:
        pass


def test_any_provider_works_through_the_interface():
    class Echo(LLMProvider):
        name = 'echo'

        def complete(self, system, user):
            assert 'one short sentence' in system and '"section": "C"' in user
            return 'In section C,  a new object (#42)\nis here.'
    text, source = Narrator(Echo()).narrate(facts_for(row('detected'), ZONES))
    assert (text, source) == ('In section C, a new object (#42) is here.', 'echo')


class FakeMessages:
    def __init__(self, reply=None, error=None, stop='end_turn'):
        self.reply, self.error, self.stop, self.calls = reply, error, stop, []

    def create(self, **kw):
        self.calls.append(kw)
        if self.error:
            raise self.error
        return SimpleNamespace(stop_reason=self.stop,
                               content=[SimpleNamespace(type='thinking', thinking=''),
                                        SimpleNamespace(type='text', text=self.reply)])


def fake_client(messages):
    return SimpleNamespace(beta=SimpleNamespace(messages=messages))


def claude(messages):
    return ClaudeProvider(client=fake_client(messages), log=lambda *_: None)


def test_claude_sentence_is_used_and_request_is_sane():
    msgs = FakeMessages(reply='In section C, a suspicious object (#42)\nhas been spotted.')
    text, source = Narrator(claude(msgs)).narrate(facts_for(row('detected'), ZONES))
    assert source == 'claude'
    assert text == 'In section C, a suspicious object (#42) has been spotted.'   # one line
    kw = msgs.calls[0]
    assert kw['model'] == 'claude-opus-5-5' and kw['output_config'] == {'effort': 'low'}
    assert kw['fallbacks'] == 'default'
    assert '"section": "C"' in kw['messages'][0]['content']


def test_claude_refusal_or_error_falls_back_to_the_template():
    n = Narrator(claude(FakeMessages(reply='', stop='refusal')))
    assert n.narrate(facts_for(row('detected'), ZONES))[1] == 'template'
    n = Narrator(claude(FakeMessages(error=RuntimeError('boom'))))
    text, source = n.narrate(facts_for(row('seen_again', level=2), ZONES))
    assert source == 'template' and 'still there' in text


def test_real_sdk_without_credentials_means_templates(monkeypatch):
    """With the anthropic package installed but no key, the provider switches off at
    startup (the SDK client itself would only fail on the first request)."""
    for var in ('ANTHROPIC_API_KEY', 'ANTHROPIC_AUTH_TOKEN', 'ANTHROPIC_PROFILE'):
        monkeypatch.delenv(var, raising=False)
    n = Narrator(make_provider('claude', log=lambda *_: None))
    assert not n.llm_enabled
    assert 'credentials' in n.llm_off_reason or 'not installed' in n.llm_off_reason
    assert n.narrate(facts_for(row('detected'), ZONES)) == \
        ('In section C, a suspicious object (#42) has been detected.', 'template')


def test_real_sdk_with_a_key_is_enabled(monkeypatch):
    monkeypatch.setenv('ANTHROPIC_API_KEY', 'sk-test-not-a-real-key')
    p = make_provider('claude', log=lambda *_: None)
    if p.sdk is None:
        return                                             # package not installed here
    assert Narrator(p).llm_enabled and p.describe() == 'claude (claude-opus-5-5, effort low)'
