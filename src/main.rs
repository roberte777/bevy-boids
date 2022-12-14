use std::slice;

use bevy::prelude::*;
use bevy_prototype_lyon::prelude::*;
use rand::prelude::*;
const ALIGNMENT: f32 = 7.;
const COHESION: f32 = 0.3;
const SEPARATION: f32 = 0.5;
const FLEE: f32 = 10000.;
const CHASE: f32 = 1.;
const TIME_STEP: f32 = 1. / 60.;
pub struct WinSize {
    pub w: f32,
    pub h: f32,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(ShapePlugin)
        .add_startup_system(setup_system)
        .add_system(update_boids)
        .run();
}

enum BoidType {
    Predator,
    Prey,
}

#[derive(Component)]
struct Boid {
    boid_type: BoidType,
    velocity: Vec2,
    acceleration: Vec2,
    max_force: f32,
    max_speed: f32,
    perception_radius: f32,
    position: Vec2,
}

#[derive(Component)]
struct Position;

#[derive(Component)]
struct Velocity;

#[derive(Component)]
struct Acceleration;

fn setup_system(mut commands: Commands, mut windows: ResMut<Windows>) {
    // capture window size
    let window = windows.get_primary_mut().unwrap();
    let (win_w, win_h) = (window.width(), window.height());
    let win_size = WinSize { w: win_w, h: win_h };
    commands.insert_resource(win_size);
    commands.spawn_bundle(Camera2dBundle::default());
    // spawn boids
    let mut rng = rand::thread_rng();
    for _ in 0..100 {
        let boid_type = if rng.gen::<f32>() < 0.1 {
            BoidType::Predator
        } else {
            BoidType::Prey
        };
        let x = rng.gen_range(-win_w / 2.0..win_w / 2.) - 100.;
        let y = rng.gen_range(-win_h / 2.0..win_h / 2.) - 100.;
        let shape = shapes::RegularPolygon {
            sides: 3,
            feature: shapes::RegularPolygonFeature::Radius(10.0),
            ..shapes::RegularPolygon::default()
        };
        let boid = Boid {
            velocity: Vec2::new(rng.gen_range(-1..1) as f32, rng.gen_range(-1..1) as f32),
            acceleration: Vec2::new(0., 0.),
            position: Vec2::new(x, y),
            max_force: match boid_type {
                BoidType::Predator => 0.8,
                BoidType::Prey => 0.7,
            },
            max_speed: match boid_type {
                BoidType::Predator => 3.,
                BoidType::Prey => 5.,
            },
            perception_radius: match boid_type {
                BoidType::Predator => 100.0,
                BoidType::Prey => 50.0,
            },
            boid_type,
        };
        commands
            .spawn_bundle(GeometryBuilder::build_as(
                &shape,
                DrawMode::Outlined {
                    fill_mode: FillMode::color(match boid.boid_type {
                        BoidType::Predator => Color::RED,
                        BoidType::Prey => Color::BLUE,
                    }),
                    outline_mode: StrokeMode::new(Color::BLACK, 1.0),
                },
                Transform::from_translation(Vec3::new(x, y, 0.)),
            ))
            .insert(boid)
            .insert(Position)
            .insert(Velocity)
            .insert(Acceleration);
    }
}
fn alignment(boids: &[&Mut<'_, Boid>], boid: &Boid) -> Vec2 {
    let mut steering = Vec2::new(0., 0.);
    let mut count = 0;
    for other in boids {
        let d = boid.position.distance(other.position);
        if d > 0. && d < boid.perception_radius {
            steering += other.velocity;
            count += 1;
        }
    }
    if count > 0 {
        steering /= count as f32;
        steering = steering.normalize() * boid.max_speed;
        steering -= boid.velocity;
    }
    steering * ALIGNMENT
}
fn cohesion(boids: &[&Mut<'_, Boid>], boid: &Boid) -> Vec2 {
    let mut steering = Vec2::new(0., 0.);
    let mut count = 0;
    for other in boids {
        let d = boid.position.distance(other.position);
        if d > 0. && d < boid.perception_radius {
            steering += other.position;
            count += 1;
        }
    }
    if count > 0 {
        steering /= count as f32;
        steering -= boid.position;
        steering = steering.normalize() * boid.max_speed;
        steering -= boid.velocity;
    }
    steering * COHESION
}

fn separation(boids: &[&Mut<'_, Boid>], boid: &Boid) -> Vec2 {
    let mut steering = Vec2::new(0., 0.);
    let mut count = 0;
    for other in boids {
        let d = boid.position.distance(other.position);
        if d > 0. && d < boid.perception_radius {
            let diff = boid.position - other.position;
            steering += diff.normalize() / d;
            count += 1;
        }
    }
    if count > 0 {
        steering /= count as f32;
        steering = steering.normalize() * boid.max_speed;
        steering -= boid.velocity;
    }
    steering * SEPARATION
}

fn flee(boids: &[&Mut<'_, Boid>], boid: &Boid) -> Vec2 {
    let mut steering = Vec2::new(0., 0.);
    for other in boids {
        if matches!(other.boid_type, BoidType::Predator) {
            let d = boid.position.distance(other.position);
            if d > 0. && d < boid.perception_radius {
                let diff = boid.position - other.position;
                steering += diff.normalize() / d;
            }
        }
    }

    // steering = steering.normalize() * boid.max_speed;
    // steering -= boid.velocity;
    steering * FLEE
}
fn chase(boids: &[&Mut<'_, Boid>], boid: &Boid) -> Vec2 {
    let mut steering = Vec2::new(0., 0.);
    for other in boids {
        if matches!(other.boid_type, BoidType::Prey) {
            let d = boid.position.distance(other.position);
            if d > 0. && d < boid.perception_radius {
                steering += other.position;
                steering -= boid.position;
                steering = steering.normalize() * boid.max_speed;
                steering -= boid.velocity;
                break;
            }
        }
    }
    steering * CHASE
}

fn update_boids(
    _commands: Commands,
    mut query: Query<(&mut Boid, &mut Transform)>,
    win_size: Res<WinSize>,
) {
    let slice = &mut query.iter_mut().collect::<Vec<_>>()[..];
    for i in 0..slice.len() {
        let (mid, tail) = slice.split_at_mut(i + 1);
        let (head, element) = mid.split_at_mut(i);
        let (boid, transform) = &mut element[0];
        let mut others = Vec::new();
        others.extend(head.iter().map(|(b, _)| b));
        others.extend(tail.iter().map(|(b, _)| b));
        let temp = others.as_slice();

        let mut steering = Vec2::new(0., 0.);
        if matches!(boid.boid_type, BoidType::Predator) {
            // predators chase Prey
            steering += separation(
                temp.iter()
                    .filter_map(|b| {
                        if matches!(b.boid_type, BoidType::Predator) {
                            Some(b.to_owned())
                        } else {
                            None
                        }
                    })
                    .collect::<Vec<_>>()
                    .as_slice(),
                boid,
            );
            steering += chase(temp, boid);
        } else {
            //Prey avoid predators and flock
            steering += flee(temp, boid);
            steering += alignment(temp, boid);
            steering += cohesion(temp, boid);
            steering += separation(temp, boid);
        }

        steering = steering.clamp_length_max(boid.max_force);

        boid.acceleration += steering;
        let acceleration = boid.acceleration;
        boid.velocity += acceleration;
        boid.velocity = boid.velocity.clamp_length_max(boid.max_speed);
        let velocity = boid.velocity;
        boid.position += velocity;
        boid.acceleration = Vec2::new(0., 0.);

        transform.translation.x = boid.position.x;
        transform.translation.y = boid.position.y;

        //// wrap around and reverse velocity
        if transform.translation.x > win_size.w / 2. {
            boid.position.x = -win_size.w / 2.;
        }
        if transform.translation.x < -win_size.w / 2. {
            boid.position.x = win_size.w / 2.;
        }
        if transform.translation.y > win_size.h / 2. {
            boid.position.y = -win_size.h / 2.;
        }
        if transform.translation.y < -win_size.h / 2. {
            boid.position.y = win_size.h / 2.;
        }

        //// update rotation
        let angle = boid.velocity.y.atan2(boid.velocity.x);
        transform.rotation = Quat::from_rotation_z(angle);
    }
}
