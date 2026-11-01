use std::marker::PhantomData;

use crate::{MoveTo, RobotResult, Space, SpaceMap};

/// Node adapter for a one-shot [`MoveTo<S>`] command.
///
/// The node borrows the robot with `&mut robot >> motion.resources.0`.
/// Input carries only the target, and Output carries only the motion result.
/// The resource owner retains the robot on success and failure.
pub struct MotionNode<R, S> {
    _types: PhantomData<fn(R) -> S>,
}

impl<R, S> MotionNode<R, S> {
    pub fn new() -> Self {
        Self { _types: PhantomData }
    }
}

impl<R, S> Default for MotionNode<R, S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<R, S> ::roplat::Node for MotionNode<R, S>
where
    R: MoveTo<S> + Send,
    S: Space + Send,
    S::Value: Send,
{
    type Input = S::Value;
    type Output = RobotResult<()>;
    type Resources = (R,);

    async fn process<'call>(
        &'call mut self,
        target: Self::Input,
        (robot,): (&'call mut R,),
    ) -> Self::Output {
        robot.move_to(target)
    }
}

/// Node adapter for a typed [`SpaceMap<From, To>`].
pub struct SpaceMapNode<M, From, To> {
    model: M,
    _space: PhantomData<(From, To)>,
}

impl<M, From, To> SpaceMapNode<M, From, To> {
    pub fn new(model: M) -> Self {
        Self { model, _space: PhantomData }
    }

    pub fn into_inner(self) -> M {
        self.model
    }
}

impl<M, From, To> ::roplat::Node for SpaceMapNode<M, From, To>
where
    M: SpaceMap<From, To> + Send + Sync,
    From: Space + Send + Sync,
    To: Space + Send + Sync,
    From::Value: Send,
    To::Value: Send,
{
    type Input = From::Value;
    type Output = RobotResult<To::Value>;
    type Resources = ();

    async fn process(&mut self, input: Self::Input, _resources: ()) -> Self::Output {
        self.model.map(input)
    }
}

impl<R, S> ::roplat::Lifecycle for MotionNode<R, S> {
    type Error = ::roplat::RoplatError;
}

impl<M, From, To> ::roplat::Lifecycle for SpaceMapNode<M, From, To> {
    type Error = ::roplat::RoplatError;
}
