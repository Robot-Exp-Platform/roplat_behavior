//! Statuses, their collection and aggregation.

use std::fmt;

/// Health of one observed thing, ordered from best to worst.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Level {
    Ok,
    Warn,
    Error,
    /// No recent information: the check cannot say.
    Stale,
}

impl fmt::Display for Level {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Level::Ok => "OK",
            Level::Warn => "WARN",
            Level::Error => "ERROR",
            Level::Stale => "STALE",
        })
    }
}

/// The verdict of one check: a level, a message for people, and the
/// measured values behind it.
#[derive(Clone, Debug, PartialEq)]
pub struct Status {
    /// Slash-separated, e.g. `arm/state`; [`Aggregator`] groups by prefix.
    pub name: String,
    pub level: Level,
    pub message: String,
    pub values: Vec<(String, String)>,
}

impl Status {
    pub fn new(name: &str, level: Level, message: impl Into<String>) -> Self {
        Self {
            name: name.to_owned(),
            level,
            message: message.into(),
            values: Vec::new(),
        }
    }

    pub fn with(mut self, key: &str, value: impl fmt::Display) -> Self {
        self.values.push((key.to_owned(), value.to_string()));
        self
    }

    pub fn value(&self, key: &str) -> Option<&str> {
        self.values
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{}] {}: {}", self.level, self.name, self.message)?;
        for (k, v) in &self.values {
            write!(f, "; {k}={v}")?;
        }
        Ok(())
    }
}

/// Named status sources polled together, the role of `diagnostic_updater`.
pub struct Updater<'a> {
    // dyn(c): status sources of different types are kept in one list.
    sources: Vec<(String, Box<dyn FnMut() -> Status + Send + 'a>)>,
}

impl<'a> Updater<'a> {
    pub fn new() -> Self {
        Self { sources: Vec::new() }
    }

    /// Adds a source; its statuses are renamed to `name`.
    pub fn add(&mut self, name: &str, source: impl FnMut() -> Status + Send + 'a) -> &mut Self {
        self.sources.push((name.to_owned(), Box::new(source)));
        self
    }

    /// One status per source, in the order added.
    pub fn update(&mut self) -> Vec<Status> {
        self.sources
            .iter_mut()
            .map(|(name, source)| Status { name: name.clone(), ..source() })
            .collect()
    }
}

impl Default for Updater<'_> {
    fn default() -> Self {
        Self::new()
    }
}

/// Groups statuses by name prefix into one status per group, the role of
/// `diagnostic_aggregator`: a group's level is the worst of its members',
/// and its values say which member is at which level.
#[derive(Clone, Debug, Default)]
pub struct Aggregator {
    groups: Vec<(String, String)>,
}

impl Aggregator {
    pub fn new() -> Self {
        Self::default()
    }

    /// Statuses whose name starts with `prefix` form the group `name`.
    /// The first matching group wins; unmatched statuses form `other`.
    pub fn group(mut self, name: &str, prefix: &str) -> Self {
        self.groups.push((name.to_owned(), prefix.to_owned()));
        self
    }

    /// One status per group that has members, in the order the groups were
    /// declared, then `other` if anything was left.
    pub fn aggregate(&self, statuses: &[Status]) -> Vec<Status> {
        let mut members: Vec<Vec<&Status>> = vec![Vec::new(); self.groups.len() + 1];
        for status in statuses {
            let k = self
                .groups
                .iter()
                .position(|(_, prefix)| status.name.starts_with(prefix.as_str()))
                .unwrap_or(self.groups.len());
            members[k].push(status);
        }
        let names = self
            .groups
            .iter()
            .map(|(name, _)| name.as_str())
            .chain(["other"]);
        names
            .zip(members)
            .filter(|(_, members)| !members.is_empty())
            .map(|(name, members)| summarise(name, &members))
            .collect()
    }

    /// The worst level among `statuses`; [`Level::Ok`] for none.
    pub fn overall(statuses: &[Status]) -> Level {
        statuses.iter().map(|s| s.level).max().unwrap_or(Level::Ok)
    }
}

fn summarise(name: &str, members: &[&Status]) -> Status {
    let level = members.iter().map(|s| s.level).max().unwrap_or(Level::Ok);
    let not_ok: Vec<&str> = members
        .iter()
        .filter(|s| s.level != Level::Ok)
        .map(|s| s.name.as_str())
        .collect();
    let message = if not_ok.is_empty() {
        format!("{} OK", members.len())
    } else {
        format!(
            "{} of {} not OK: {}",
            not_ok.len(),
            members.len(),
            not_ok.join(", ")
        )
    };
    let mut status = Status::new(name, level, message);
    for member in members {
        status = status.with(&member.name, member.level);
    }
    status
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levels_order_from_best_to_worst() {
        assert!(
            Level::Ok < Level::Warn && Level::Warn < Level::Error && Level::Error < Level::Stale
        );
    }

    #[test]
    fn aggregator_groups_by_prefix_and_keeps_the_worst() {
        let statuses = [
            Status::new("arm/state", Level::Ok, "100 Hz"),
            Status::new("arm/command", Level::Warn, "82 Hz"),
            Status::new("camera/rgb", Level::Ok, "30 Hz"),
            Status::new("planner", Level::Error, "no events"),
        ];
        let groups = Aggregator::new()
            .group("arm", "arm/")
            .group("camera", "camera/")
            .aggregate(&statuses);
        assert_eq!(groups.len(), 3);
        assert_eq!(
            (groups[0].name.as_str(), groups[0].level),
            ("arm", Level::Warn)
        );
        assert_eq!(groups[0].message, "1 of 2 not OK: arm/command");
        assert_eq!(groups[0].value("arm/state"), Some("OK"));
        assert_eq!(
            (groups[1].name.as_str(), groups[1].level),
            ("camera", Level::Ok)
        );
        assert_eq!(
            (groups[2].name.as_str(), groups[2].level),
            ("other", Level::Error)
        );
        assert_eq!(Aggregator::overall(&statuses), Level::Error);
        assert_eq!(Aggregator::overall(&[]), Level::Ok);
    }

    #[test]
    fn updater_polls_sources_by_name() {
        let mut count = 0;
        let mut updater = Updater::new();
        updater.add("counter", move || {
            count += 1;
            Status::new("ignored", Level::Ok, "counting").with("count", count)
        });
        assert_eq!(updater.update()[0].value("count"), Some("1"));
        let second = updater.update();
        assert_eq!(
            (second[0].name.as_str(), second[0].value("count")),
            ("counter", Some("2"))
        );
        assert_eq!(second[0].to_string(), "[OK] counter: counting; count=2");
    }
}
