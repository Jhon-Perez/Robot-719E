use alloc::vec::Vec;
use evian::math::Vec2;

use super::command::Command;


fn vec2(arg: &str) -> Result<Vec2<f64>, &'static str> {
    let arg = arg.trim();
    if !arg.starts_with('(') || !arg.ends_with(')') {
        return Err("Invalid Vec2 format");
    }

    let coords: Vec<&str> = arg[1..arg.len() - 1].split(',').map(|s| s.trim()).collect();
    if coords.len() != 2 {
        return Err("Vec2 must contain two values");
    }

    let x = coords[0]
        .parse::<f64>()
        .map_err(|_| "Invalid float for Vec2")?;
    let y = coords[1]
        .parse::<f64>()
        .map_err(|_| "Invalid float for Vec2")?;

    Ok(Vec2::new(x, y))
}

pub fn single_vec2(args: &[&str]) -> Result<Vec2<f64>, &'static str> {
    if args.len() != 1 {
        return Err("Expected one argument for Vec2");
    }
    vec2(args[0])
}

pub fn multiple_vec2(args: &[&str], expected: usize) -> Result<Vec<Vec2<f64>>, &'static str> {
    if args.len() != expected {
        return Err("Incorrect number of Vec2 arguments");
    }

    args.iter().map(|&arg| vec2(arg)).collect()
}

pub fn single_f64(args: &[&str]) -> Result<f64, &'static str> {
    if args.len() != 1 {
        return Err("Expected one float argument");
    }
    args[0].parse::<f64>().map_err(|_| "Invalid float argument")
}

pub fn single_u64(args: &[&str]) -> Result<u64, &'static str> {
    if args.len() != 1 {
        return Err("Expected one integer argument");
    }
    args[0]
        .parse::<u64>()
        .map_err(|_| "Invalid integer argument")
}

pub fn pose(args: &[&str]) -> Result<Command, &'static str> {
    if args.len() != 2 {
        return Err("Pose requires two arguments: (x,y) and angle");
    }

    let position = vec2(args[0])?;
    let angle = single_f64(&[args[1]])?;

    Ok(Command::Pose(position, angle))
}
