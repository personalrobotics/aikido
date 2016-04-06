## State Space Concept

This directory contains `StateSpace` intended to represent the configuration
space of an arbitrary robot and provides utilities for mapping between a
`StateSpace` and the `DegreeOfFreedom`s of a DART `MetaSkeleton`. The following
`StateSpace` types are included:

- `RealVectorStateSpace` an `n`-dimensional real vector space
- `SO2StateSpace` the space of planar rotations
- `SO3StateSpace` the space of spatial rotations
- `CompoundStateSpace` a Cartesian product of other `StateSpace`s

Each of these `StateSpace` classes is paired with a `StateSpace::State` class
that represents an element of that state. The `StateSpace` stores *all*
information that is common between `State`s to keep that class as lightweight
as possible. This is critical because motion planning algorithms tend to create
large numbers of `State`s.


### Creating and Destroying `State`s

For a *fixed-size `StateSpace`*, such as `SO2StateSpace` or `SO3StateSpace`,
you may directly instantiate and modify the `State` type:
```c++
SO2StateSpace::State fixed_state;
fixed_state.setAngle(M_PI);
```

This is not possible on *variable-size `StateSpace`*, such as
`RealVectorStateSpace` and `CompoundStateSpace`: the `State` does not know its
own size! You must create *and* modify variable-length `State`s through their
`StateSpace` class:
```c++
RealVectorStateSpace space(2);
auto state = static_cast<RealVectorStateSpace::State*>(space.allocateState());
space.getValue(*state) = Eigen::Vector2d(1., 2.);
```

We provide a `StateHandle` class to automate this operation by storing an
internal pointer to the `StateSpace` class associated with the `State` that it
wraps. This convenience comes at the cost of additional memory usage, so
`StateHandle` is not recommended for performance-critical applications:
```c++
RealVectorStateSpace::StateHandle state_handle(&space, state);
state_handle.getValue() = Eigen::Vector2d(3., 4.);
```

In both cases, you must remember to call `space.deallocateState(state)` to
avoid leaking memory. We provide a `ScopedState` class that combines the
functionality of `StateHandle` with RAII memory management. Similar to
`StateHandle`, this convenience comes at the cost of additional memory usage:
```c++
RealVectorStateSpace::ScopedState scoped_state(&space);
state_handle.getValue() = Eigen::Vector2d(1., 2.);
```

Our general advice is:

- Avoid creating large numbers of `StateHandle` or `ScopedState`
- Use `ScopedState` in all other situations to avoid manual resource management
- Guarantee that `StateSpace` and `State` outlive all of their `StateHandle`s


### Working with `CompoundStateSpace`

`CompoundStateSpace` represents the Cartesian product of an arbitrary number of
heterogeneous `StateSpace`s, known as *subspaces*:
```c++
CompoundStateSpace compound_space({
  std::make_shared<SO2StateSpace>(),
  std::make_shared<RealVectorStateSpace>(2)
});
CompoundStateSpace::ScopedState compound_handle(compound_space);
CompoundStateSpace::State* compound_state = compound_handle.getState();
```

This class also provides member functions for retrieving a subspace by index.
Accessing a subspace with incorrect type will result in a runtime error (in
debug mode) or crash (in release mode):
```c++
compound_space.getSubSpace<SO2StateSpace>(0); // OK
compound_space.getSubSpace<RealVectorStateSpace>(1); // OK
compound_space.getSubSpace<StateSpace>(1); // OK, inherits from StateSpace
compound_space.getSubSpace<SO2StateSpace>(1); // ERROR, type mismatch
```

Since `CompoundStateSpace` is variable-length*, it also provides member
functions for accessing substates by index. Just as above, accessing an
incorrect type will result in a runtime error or crash:
```c++
compound_space.getSubState<SO2StateSpace>(compound_state, 0); // OK
compound_space.getSubState<RealVectorStateSpace>(compound_state, 1); // OK
compound_space.getSubState<StateSpace>(1, 1); // OK, inherits from StateSpace
compound_space.getSubState<SO2StateSpace>(1, 1); // ERROR, type mismatch
```

This is inconvienent when working with nested variable-length state spaces. For
example, the following code is required to set the real vector component of
`compound_state`:
```c++
compound_space.getSubSpace<RealVectorStateSpace>(1).setValue(
  compound_space.getSubState<RealVectorStateSpace>(compound_state, 1),
  Eigen::Vector2d(3., 4.));
```

It is significantly less verbose to use the `StateHandle` wrapper:
```c++
compound_handle.getSubStateHandle<RealVectorStateSpace>(1).getValue()
  = Eigen::Vector2d(3., 4.);
```


## `StateSpace` formal definition

The type `SS` satisfies the `StateSpace` concept iff:

- The type `SS` inherits from `aikido::statespace::StateSpace`
- The type `SS::State` inherits from `State`
- The type `SS::StateHandle` inherits from `StateHandle<SS, SS::State>`
- The type `SS::StateHandleConst` inherits from `StateHandle<SS, const SS::State>`
- The type `SS::ScopedState` is `ScopedState<SS:StateHandle>`
- The type `SS::ScopedStateConst` is `ScopedState<SSStateHandleConst>`

