## State Space Concept

This directory contains `StateSpace` intended to represent the configuration
space of an arbitrary robot and provides utilities for mapping between a
`StateSpace` and the `DegreeOfFreedom`s of a DART `MetaSkeleton`. The following
`StateSpace` types are included:

- `Rn` an `n`-dimensional real vector space
- `SO2` the space of planar rotations
- `SO3` the space of spatial rotations
- `CartesianProduct` of other `StateSpace`s

Each of these `StateSpace` classes is paired with a `StateSpace::State` class
that represents an element of that state. The `StateSpace` stores *all*
information that is common between `State`s to keep that class as lightweight
as possible. This is critical because motion planning algorithms tend to create
large numbers of `State`s.


### Creating and Destroying `State`s

For a *fixed-size `StateSpace`, such as `SO2` or `SO3`,
you may directly instantiate and modify the `State` type:
```c++
SO2::State fixed_state;
fixed_state.fromAngle(M_PI);
```

This is not possible on *variable-size `StateSpace`*, such as
`Rn` and `CartesianProduct`: the `State` does not know its
own size! You must create *and* modify variable-length `State`s through their
`StateSpace` class:
```c++
Rn space(2);
auto state = static_cast<Rn::State*>(space.allocateState());
space.setValue(state, Eigen::Vector2d(1., 2.));
```

We provide a `StateHandle` class to automate this operation:
```c++
Rn::StateHandle state_handle(&space, state);
state_handle.setValue(Eigen::Vector2d(3., 4.));
```

In both cases, you must remember to call `space.deallocateState(state)` to
avoid leaking memory. We provide a `ScopedState` class that combines the
functionality of `StateHandle` with RAII memory management:
```c++
Rn::ScopedState scoped_state(&space);
scoped_state.setValue(Eigen::Vector2d(1., 2.));
```

For typical use, we *strongly* recommend using `ScopedState` to avoid manual
memory management. However, the convenience of `StateHandle` and `ScopedState`
comes at the cost of memory overhead. We suggest avoiding these classes, by
using the operations provided by `StateSpace`, in performance-critical code.
This is especially important when creating a large number of states, e.g. when
implementing a sample-based motion planner.


### Working with `CartesianProduct`

`CartesianProduct` represents the Cartesian product of an arbitrary number of
heterogeneous `StateSpace`s, known as *subspaces*:
```c++
CartesianProduct compound_space({
  std::make_shared<SO2>(),
  std::make_shared<Rn>(2)
});
CartesianProduct::ScopedState compound_handle(compound_space);
CartesianProduct::State* compound_state = compound_handle.getState();
```

This class also provides member functions for retrieving a subspace by index.
Accessing a subspace with incorrect type will result in a runtime error (in
debug mode) or crash (in release mode):
```c++
compound_space.getSubspace<SO2>(0); // OK
compound_space.getSubspace<Rn>(1); // OK
compound_space.getSubspace<StateSpace>(1); // OK, inherits from StateSpace
compound_space.getSubspace<SO2>(1); // ERROR, type mismatch
```

Since `CartesianProduct` is variable-length*, it also provides member
functions for accessing substates by index. Just as above, accessing an
incorrect type will result in a runtime error or crash:
```c++
compound_space.getSubState<SO2>(compound_state, 0); // OK
compound_space.getSubState<Rn>(compound_state, 1); // OK
compound_space.getSubState<StateSpace>(compound_state, 1); // OK
compound_space.getSubState<SO2>(compound_state, 1); // ERROR, type mismatch
```

This is inconvienent when working with nested variable-length state spaces. For
example, the following code is required to set the real vector component of
`compound_state`:
```c++
compound_space.getSubspace<Rn>(1).setValue(
  compound_space.getSubState<Rn>(compound_state, 1), Eigen::Vector2d(3., 4.));
```

It is significantly less verbose to use the `StateHandle` wrapper:
```c++
compound_handle.getSubStateHandle<Rn>(1).setValue(Eigen::Vector2d(3., 4.));
```


### Implementing your own `StateSpace`

It is straightforward to implement your own `StateSpace`. The type `SS`
satisfies the `StateSpace` concept iff:

- The type `SS` inherits from `aikido::statespace::StateSpace`
- The type `SS::State` has no virtual functions
- The type `SS::State` inherits from `aikido::statespace::StateSpace::State`
- The type `SS::StateHandle` inherits from
  `aikido::statespace::StateHandle<SS, SS::State>`
- The type `SS::StateHandleConst` inherits from
  `aikido::statespace::StateHandle<SS, const SS::State>`
- The type `SS::ScopedState` is
  `aikido::statespace::ScopedState<SS:StateHandle>`
- The type `SS::ScopedStateConst` is
  `aikido::statespace::ScopedState<SSStateHandleConst>`

Inheriting from `aikido::statespace::StateSpace` requires implementing several
pure-`virtual` methods:

- `getStateSizeInBytes` returns the amount of memory required, in bytes, by
  `allocateStateInBuffer`. For a fixed-size state, this is typically
  `sizeof(MyState)`.

- `allocateStateInBuffer` constructs a `State` object in the buffer passed
  as an argument. For a fixed size state, this is typically implemented using
  the placement`-new` operator: `new (_buffer) MyState`.

- `freeStateInBuffer` frees memory associated with a `State` object previously
  allocated by `allocateStateInBuffer`, but *not the buffer itself*. For a
  fixed size state, this is typically implemented by explicitly calling the
  destructor: `_state->~MyState()`.

- `compose` applies the multiplication operator associated with the
  `StateSpace`'s Lie group to `_state1` and `_state2`, storing the result in
  the output parameter `_out`. For a real vector space, this operator
  implements vector addition.
