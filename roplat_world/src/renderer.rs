use anyhow::Result;

pub trait Renderer {
    // fn start(&mut self) {} // open a window, a connection or a recording stream
    // fn shutdown(&mut self) {} // tear down
}

pub trait AttachFrom<T> {
    fn attach_from(self, from: &mut T) -> Result<()>;
}
