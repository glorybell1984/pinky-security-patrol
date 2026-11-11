"""Turn patrol events into one short sentence for the operator.

    "In section A, a suspicious object (#42) has been detected."
    "In section A, object #42 is still there and has not moved."
    "In section A, someone just picked up object #42 and I am following it."

Claude writes the sentence when an API key (or `ant auth login` profile) and the
network are available; otherwise, or on any error or timeout, a fixed template
does. The robot never waits for the LLM: the ROS node calls narrate() from a
worker thread.

Pure Python apart from the optional `anthropic` package (imported lazily), so
the event -> facts -> sentence logic is unit tested without ROS or a network.
"""
import json
import time

# event name -> what happened, in words the model can rephrase (and the template uses)
EVENTS = {
    'detected': 'a new suspicious object was detected (warning level 1)',
    'seen_again': 'the same object is still at the same place on a later patrol pass, '
                  'it has not moved (warning level 2); the robot stops and signals',
    'follow_start': 'the object moved, someone probably picked it up, and the robot is '
                    'now following it (warning level 3)',
    'follow_skipped': 'the object is moving but the robot cannot follow it right now',
    'lost_track': 'the robot lost sight of the object it was following and goes back '
                  'to patrol',
    'relocated': 'the object turned up at a new place',
    'removed': 'the object is no longer at its logged place',
    'acknowledged': 'the operator checked the object and cleared it',
    'go_charge': 'the robot is going back to its charging dock',
    'charger_reached': 'the robot is on its dock',
    'battery_low': 'the battery is low',
    'battery_critical': 'the battery is critically low',
}

TEMPLATES = {
    'detected': 'a suspicious object (#{id}) has been detected.',
    'seen_again': 'object #{id} is still there and has not moved.',
    'follow_start': 'someone just picked up object #{id} and I am following it.',
    'follow_skipped': 'object #{id} is moving, but I cannot follow it right now.',
    'lost_track': 'I lost sight of object #{id} and am going back to patrol.',
    'relocated': 'object #{id} has turned up in a new place.',
    'removed': 'object #{id} is no longer at its place.',
    'acknowledged': 'object #{id} was checked and cleared by the operator.',
    'go_charge': 'I am heading back to my dock.',
    'charger_reached': 'I am on my dock.',
    'battery_low': 'my battery is low.',
    'battery_critical': 'my battery is critically low.',
}

OBJECT_EVENTS = {'detected', 'seen_again', 'follow_start', 'follow_skipped', 'lost_track',
                 'relocated', 'removed', 'acknowledged'}

SYSTEM_PROMPT = (
    'You are the voice of a small security patrol robot, reporting to a human operator '
    'on a dashboard. For each event you get the facts as JSON. Reply with exactly one '
    'short sentence (at most 22 words), first person, plain words, no emojis, no '
    'coordinates or numbers other than the object number. When a section is given, start '
    'with "In section <name>," and use the section name exactly as given. Say only what '
    'the facts say; do not invent people, causes or details. Keep it calm and factual; '
    'a higher warning level may sound more urgent. Use earlier reports only to avoid '
    'repeating yourself word for word.'
)


def facts_for(row, zone_map):
    """Facts for one /fleet/events row (dict), or None if this event is not narrated."""
    event = row.get('event', '')
    if event not in EVENTS:
        return None
    marker = row.get('marker_id', '')
    if event in OBJECT_EVENTS:
        x, y = _num(row.get('x_map')), _num(row.get('y_map'))
        if x is None:                                 # some events carry no object position
            x, y = _num(row.get('robot_x')), _num(row.get('robot_y'))
    else:
        x, y = _num(row.get('robot_x')), _num(row.get('robot_y'))
    zone = zone_map.zone_at(x, y) if zone_map is not None else None
    level = row.get('warning_level', '')
    return {
        'event': event,
        'what_happened': EVENTS[event],
        'object': f'#{marker}' if marker not in ('', None) else None,
        'warning_level': level if level not in ('', None) else None,
        'section': zone.name if zone else None,
        'section_description': zone.description if zone and zone.description else None,
        'patrol_pass': row.get('patrol_pass'),
    }


def template_sentence(facts):
    marker = (facts.get('object') or '#?').lstrip('#')
    body = TEMPLATES.get(facts['event'], facts['what_happened'] + '.').format(id=marker)
    if facts.get('section'):
        return f'In section {facts["section"]}, {body}'
    return body[0].upper() + body[1:]


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


class Narrator:
    """Writes the sentence with Claude, falling back to the template."""

    def __init__(self, use_llm=True, model='claude-opus-5-5', effort='low',
                 timeout=8.0, language='English', client=None, log=print):
        self.model = model
        self.effort = effort
        self.language = language
        self.log = log
        self.recent = []                  # last sentences, for variety
        self.llm_off_reason = None
        self.backoff_until = 0.0
        self.client = client
        if not use_llm:
            self.llm_off_reason = 'disabled (narrator_mode: template)'
        elif client is None:
            try:
                import anthropic                       # optional dependency
                self._anthropic = anthropic
                # Credentials: ANTHROPIC_API_KEY, ANTHROPIC_AUTH_TOKEN or an `ant auth login`
                # profile. Short timeout and one retry: a late sentence is useless.
                self.client = anthropic.Anthropic(timeout=timeout, max_retries=1)
            except ImportError:
                self.llm_off_reason = 'the anthropic package is not installed'
            except Exception as e:  # noqa: BLE001  (no credentials, bad config)
                self.llm_off_reason = f'no Claude credentials ({e.__class__.__name__})'
        if self.client is not None and not hasattr(self, '_anthropic'):
            try:
                import anthropic
                self._anthropic = anthropic
            except ImportError:
                self._anthropic = None

    @property
    def llm_enabled(self):
        return self.client is not None and self.llm_off_reason is None

    def narrate(self, facts):
        """-> (sentence, source) where source is 'claude' or 'template'."""
        text, source = None, 'template'
        if self.llm_enabled and time.monotonic() >= self.backoff_until:
            text = self._ask_claude(facts)
            if text:
                source = 'claude'
        if not text:
            text = template_sentence(facts)
        self.recent = (self.recent + [text])[-3:]
        return text, source

    def _ask_claude(self, facts):
        prompt = json.dumps({'facts': {k: v for k, v in facts.items() if v is not None},
                             'language': self.language,
                             'earlier_reports': self.recent}, ensure_ascii=False)
        try:
            # Server-side refusal fallback (beta) so a declined request still gets an
            # answer from another model; the template covers anything else.
            response = self.client.beta.messages.create(
                model=self.model,
                max_tokens=1024,
                system=SYSTEM_PROMPT,
                messages=[{'role': 'user', 'content': prompt}],
                output_config={'effort': self.effort},
                betas=['server-side-fallback-2026-07-01'],
                fallbacks='default',
            )
        except Exception as e:  # noqa: BLE001  (sorted below; the template always works)
            self._on_error(e)
            return None
        if response.stop_reason == 'refusal':
            self.log('narrator: Claude declined, using the template')
            return None
        text = ' '.join(b.text for b in response.content if b.type == 'text').strip()
        text = ' '.join(text.split())                  # one line for the dashboard
        return text or None

    def _on_error(self, e):
        a = self._anthropic
        if a is not None and isinstance(e, (a.AuthenticationError, a.PermissionDeniedError)):
            self.llm_off_reason = f'Claude rejected the credentials ({e.__class__.__name__})'
            self.log(f'narrator: {self.llm_off_reason}; templates from now on')
        elif a is not None and isinstance(e, a.NotFoundError):
            self.llm_off_reason = f'model {self.model} not found'
            self.log(f'narrator: {self.llm_off_reason}; templates from now on')
        elif a is not None and isinstance(e, a.RateLimitError):
            self.backoff_until = time.monotonic() + 30.0
            self.log('narrator: rate limited, templates for 30 s')
        elif a is not None and isinstance(e, a.APIConnectionError):   # incl. timeouts
            self.backoff_until = time.monotonic() + 10.0
            self.log('narrator: Claude not reachable (network/timeout), template used')
        else:
            self.backoff_until = time.monotonic() + 10.0
            self.log(f'narrator: Claude error {e.__class__.__name__}: {e}; template used')
