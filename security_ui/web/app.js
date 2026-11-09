// Connects to rosbridge and fills the dashboard tabs.
const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

ros.on('connection', () => { document.getElementById('connection').textContent = 'connected'; });
ros.on('close', () => { document.getElementById('connection').textContent = 'disconnected'; });

document.querySelectorAll('nav button').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('main section').forEach((s) => { s.hidden = s.id !== btn.dataset.tab; });
  });
});

const objects = new ROSLIB.Topic({
  ros, name: '/fleet/objects', messageType: 'security_interfaces/msg/TrackedObjectArray',
});
objects.subscribe((msg) => {
  // TODO: render the object registry table
  console.log('objects', msg.objects.length);
});

const events = new ROSLIB.Topic({ ros, name: '/fleet/events', messageType: 'std_msgs/msg/String' });
events.subscribe((msg) => {
  // TODO: append to the event log
  console.log('event', msg.data);
});
