use bevy::{prelude::*, render::texture};
use bevy_prototype_lyon::prelude::*;
use rand::prelude::*;
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
            boid_type,
            velocity: Vec2::new(rng.gen_range(-1..1) as f32, rng.gen_range(-1..1) as f32),
            acceleration: Vec2::new(0., 0.),
            position: Vec2::new(x, y),
            max_force: 0.1,
            max_speed: 2.,
            perception_radius: 50.,
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
    let mut total = 0;
    for other in boids {
        let d = boid.position.distance(other.position);
        if d > 0. && d < boid.perception_radius {
            steering += other.velocity;
            total += 1;
        }
    }
    if total > 0 {
        steering /= total as f32;
        steering = steering.normalize() * boid.max_speed;
        steering -= boid.velocity;
        steering = steering.clamp_length_max(boid.max_force);
    }
    steering
}
fn cohesion(boids: &[&Mut<'_, Boid>], boid: &Boid) -> Vec2 {
    let mut steering = Vec2::new(0., 0.);
    let mut total = 0;
    for other in boids {
        let d = boid.position.distance(other.position);
        if d > 0. && d < boid.perception_radius {
            steering += other.position;
            total += 1;
        }
    }
    if total > 0 {
        steering /= total as f32;
        steering -= boid.position;
        steering = steering.normalize() * boid.max_speed;
        steering -= boid.velocity;
        steering = steering.clamp_length_max(boid.max_force);
    }
    steering / 2.
}

fn separation(boids: &[&Mut<'_, Boid>], boid: &Boid) -> Vec2 {
    let mut steering = Vec2::new(0., 0.);
    let mut total = 0;
    for other in boids {
        let d = boid.position.distance(other.position);
        if d > 0. && d < boid.perception_radius {
            let diff = boid.position - other.position;
            steering += diff.normalize() / d;
            total += 1;
        }
    }
    if total > 0 {
        steering /= total as f32;
        steering = steering.normalize() * boid.max_speed;
        steering -= boid.velocity;
        steering = steering.clamp_length_max(boid.max_force);
    }
    steering / 2.
}

fn update_boids(
    mut commands: Commands,
    mut query: Query<(&mut Boid, &mut Transform)>,
    win_size: Res<WinSize>,
) {
    let slice = &mut query.iter_mut().collect::<Vec<_>>()[..];
    for i in 0..slice.len() {
        if i == slice.len() - 1 {
            break;
        }
        let (mid, tail) = slice.split_at_mut(i + 1);
        let (head, element) = mid.split_at_mut(i);
        let (boid, transform) = &mut element[0];
        //use alignment, cohesion, separation to determine steering
        //combine head and tail
        let mut others = Vec::new();
        others.extend(head.iter().map(|(b, _)| b));
        others.extend(tail.iter().map(|(b, _)| b));
        let temp = others.as_slice();

        let steering = alignment(temp, boid) + cohesion(temp, boid) + separation(temp, boid);
        boid.acceleration += steering;
        let acceleration = boid.acceleration;
        boid.velocity += acceleration;
        boid.velocity = boid.velocity.clamp_length_max(boid.max_speed);
        transform.translation += Vec3::new(boid.velocity.x, boid.velocity.y, 0.);

        // wrap around
        if transform.translation.x > win_size.w / 2. {
            transform.translation.x = -win_size.w / 2.;
        }
        if transform.translation.x < -win_size.w / 2. {
            transform.translation.x = win_size.w / 2.;
        }
        if transform.translation.y > win_size.h / 2. {
            transform.translation.y = -win_size.h / 2.;
        }
        if transform.translation.y < -win_size.h / 2. {
            transform.translation.y = win_size.h / 2.;
        }

        // update rotation
        let angle = boid.velocity.y.atan2(boid.velocity.x);
        transform.rotation = Quat::from_rotation_z(angle);

        //update position
        boid.position = Vec2::new(transform.translation.x, transform.translation.y);
    }
}
