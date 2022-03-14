# SMACH tutorial

## Getting Started with smach

```bash
$ sudo apt-get install ros-noetic-smach-ros
```

## Creating a State Machine

Creating a state

```python
  class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2']):
       # Your state initialization goes here

     def execute(self, userdata):
        # Your state execution goes here
        if xxxx:
            return 'outcome1'
        else:
            return 'outcome2'
```

Adding states to a state machine

```python
  sm = smach.StateMachine(outcomes=['outcome4','outcome5'])
  with sm:
     smach.StateMachine.add('FOO', Foo(),
                            transitions={'outcome1':'BAR',
                                         'outcome2':'outcome4'})
     smach.StateMachine.add('BAR', Bar(),
                            transitions={'outcome2':'FOO'})
```

Example

<https://raw.githubusercontent.com/rhaschke/executive_smach_tutorials/indigo-devel/examples/state_machine_simple.py>

`roscore` should be running

## Pre-defined States and Containers

State library

Container library

## Passing User Data between States

Specifying User Data

```python
  class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2'],
                        input_keys=['foo_input'],
                        output_keys=['foo_output'])

     def execute(self, userdata):
        # Do something with userdata
        if userdata.foo_input == 1:
            return 'outcome1'
        else:
            userdata.foo_output = 3
            return 'outcome2'
```

The `input_keys` list enumerates all the inputs that a state needs to run. A state declares that it expect these fields to exist in the userdata. The execute method is provided a copy of the userdata struct. The state can read from all userdata fields that it enumerates in the `input_keys` list, but it can't write to any of these fields.

The `output_keys` list enumerates all the outputs that a state provides. The state can write to all fields in the userdata struct that are enumerated in the `output_keys` list.

Connecting User Data

```python
  sm_top = smach.StateMachine(outcomes=['outcome4','outcome5'],
                          input_keys=['sm_input'],
                          output_keys=['sm_output'])
  with sm_top:
     smach.StateMachine.add('FOO', Foo(),
                            transitions={'outcome1':'BAR',
                                         'outcome2':'outcome4'},
                            remapping={'foo_input':'sm_input',
                                       'foo_output':'sm_data'})
     smach.StateMachine.add('BAR', Bar(),
                            transitions={'outcome2':'FOO'},
                            remapping={'bar_input':'sm_data',
                                       'bar_output1':'sm_output'})
```

Example

<https://raw.githubusercontent.com/eacousineau/executive_smach_tutorials/hydro-devel/smach_tutorials/examples/user_data2.py>

## Create a Hierarchical State Machine

Create some states

```python
  # State Foo
  class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2'])
     
     def execute(self, userdata):
        return 'outcome1'


  # State Bar
  class Bar(smach.State):
     def __init__(self, outcomes=['outcome1'])
     
     def execute(self, userdata):
        return 'outcome4'


  # State Bas
  class Bas(smach.State):
     def __init__(self, outcomes=['outcome3'])
     
     def execute(self, userdata):
        return 'outcome3'
```

Creating a hierarchical state machine

```python
   # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])

    # Open the container
    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'outcome3':'SUB'})

        # Create the sub SMACH state machine 
        sm_sub = smach.StateMachine(outcomes=['outcome4'])

        # Open the container 
        with sm_sub:

            # Add states to the container 
            smach.StateMachine.add('FOO', Foo(),
                                   transitions={'outcome1':'BAR', 
                                                'outcome2':'outcome4'})
            smach.StateMachine.add('BAR', Bar(),
                                   transitions={'outcome1':'FOO'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'outcome4':'outcome5'})
```

Example

<https://raw.githubusercontent.com/eacousineau/executive_smach_tutorials/hydro-devel/smach_tutorials/examples/state_machine_nesting2.py>

## Calling Actions from a State Machine (ROS)

Goal Message

* empty

```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    smach.StateMachine.add('TRIGGER_GRIPPER',
                           SimpleActionState('action_server_namespace',
                                             GripperAction),
                           transitions={'succeeded':'APPROACH_PLUG'})
```

* fixed

```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    gripper_goal = Pr2GripperCommandGoal()
    gripper_goal.command.position = 0.07
    gripper_goal.command.max_effort = 99999
    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal=gripper_goal),
                      transitions={'succeeded':'APPROACH_PLUG'})
```

* from user data

``` python
Превключи показването на номерата на редовете
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal_slots=['max_effort', 
                                                    'position']),
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'max_effort':'user_data_max',
                                 'position':'user_data_position'})
```

* callback

```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    def gripper_goal_cb(userdata, goal):
       gripper_goal = GripperGoal()
       gripper_goal.position.x = 2.0
       gripper_goal.max_effort = userdata.gripper_input
       return gripper_goal

    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal_cb=gripper_goal_cb,
                                        input_keys=['gripper_input'])
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'gripper_input':'userdata_input'})
```

Result Message

* to userdata

```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        result_slots=['max_effort', 
                                                      'position']),
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'max_effort':'user_data_max',
                                 'position':'user_data_position'})
```

* callback

```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    def gripper_result_cb(userdata, status, result):
       if status == GoalStatus.SUCCEEDED:
          userdata.gripper_output = result.num_iterations
          return 'my_outcome'

    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        result_cb=gripper_result_cb,
                                        output_keys=['gripper_output'])
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'gripper_output':'userdata_output'})
```

## Viewing State Machines (ROS)

Python2 only

```bash
$ sudo apt-get install ros-noetic-smach-viewer
```

## References

* <http://wiki.ros.org/smach/Tutorials>
* [Getting Started with smach](http://wiki.ros.org/smach/Tutorials/Getting%20Started)
* [Passing User Data between States](http://wiki.ros.org/smach/Tutorials/User%20Data)
* [Create a Hierarchical State Machine](http://wiki.ros.org/smach/Tutorials/Create%20a%20hierarchical%20state%20machine)
* [Calling Actions from a State Machine (ROS)](http://wiki.ros.org/smach/Tutorials/Calling%20Actions)
