//! roplat node recording joint states into a [`FrameBuffer`].

use std::marker::PhantomData;

use roplat_msgs::{JointSample, JointState, Stamp};

use crate::{FrameBuffer, FrameError, JointGroup, TreeError};

/// Joint positions carried by a [`JointStateNode`] input.
pub trait JointPositions<const N: usize> {
    /// `None` when the value has no positions.
    fn positions(&self) -> Option<[f64; N]>;
}

impl<const N: usize> JointPositions<N> for [f64; N] {
    fn positions(&self) -> Option<[f64; N]> {
        Some(*self)
    }
}

impl<const N: usize> JointPositions<N> for JointSample<N> {
    fn positions(&self) -> Option<[f64; N]> {
        self.q()
    }
}

/// The measured positions.
impl<const N: usize> JointPositions<N> for JointState<N> {
    fn positions(&self) -> Option<[f64; N]> {
        self.meas.q()
    }
}

/// Records the positions of one joint group at their stamp, so that every
/// link below those joints can be looked up at that instant.
///
/// The node borrows the buffer with `&mut frames >> node.resources.0`.
/// Input is `(stamp, state)`, Output the write result. A state without
/// positions is [`FrameError::MissingPositions`], never zeros. Link poses are
/// computed when they are looked up, so recording copies `N` numbers.
pub struct JointStateNode<const N: usize, P = JointState<N>> {
    group: JointGroup<N>,
    _input: PhantomData<fn(P)>,
}

impl<const N: usize, P> JointStateNode<N, P> {
    pub fn new(group: JointGroup<N>) -> Self {
        Self { group, _input: PhantomData }
    }
}

impl<const N: usize, P> ::roplat::Node for JointStateNode<N, P>
where
    P: JointPositions<N> + Send,
{
    type Resources = (FrameBuffer,);
    type Input = (Stamp, P);
    type Output = Result<(), FrameError>;

    async fn process<'call>(
        &'call mut self,
        (stamp, state): Self::Input,
        (frames,): (&'call mut FrameBuffer,),
    ) -> Self::Output {
        let q = state.positions().ok_or(FrameError::MissingPositions)?;
        frames.set_joints(self.group, stamp, &q)
    }
}

impl<const N: usize, P> ::roplat::Lifecycle for JointStateNode<N, P> {
    type Error = ::roplat::RoplatError;
}

/// Lets a graph raise a lookup or write failure with `?`; the error enters
/// the core through `RoplatError::Node`.
impl From<FrameError> for ::roplat::RoplatError {
    fn from(error: FrameError) -> Self {
        ::roplat::RoplatError::Node(Box::new(error))
    }
}

/// Lets the head of a System issue group handles with `?`.
impl From<TreeError> for ::roplat::RoplatError {
    fn from(error: TreeError) -> Self {
        ::roplat::RoplatError::Node(Box::new(error))
    }
}
