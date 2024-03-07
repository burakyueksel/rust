use std::time::{Duration, Instant};

// Struct to represent a PID controller
struct PIDController {
    kp: f64,                // Proportional gain
    ki: f64,                // Integral gain
    kd: f64,                // Derivative gain
    setpoint: f64,          // Setpoint for the controller
    integral: f64,          // Integral term for PID control
    prev_error: f64,        // Previous error for derivative term
    prev_time: Instant,     // Previous time for derivative term
}

impl PIDController {
    // Constructor for PID controller
    fn new(kp: f64, ki: f64, kd: f64, setpoint: f64) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            setpoint,
            integral: 0.0,
            prev_error: 0.0,
            prev_time: Instant::now(),
        }
    }

    // Function to compute control output
    fn compute(&mut self, current_value: f64) -> f64 {
        // Get current time
        let now = Instant::now();
        // Calculate time difference since last update
        let dt = now.duration_since(self.prev_time).as_secs_f64();
        // Calculate error
        let error = self.setpoint - current_value;

        // Compute integral term
        self.integral += error * dt;
        // Compute derivative term
        let derivative = (error - self.prev_error) / dt;

        // Calculate control output
        let output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative);

        // Update previous error and time for next iteration
        self.prev_error = error;
        self.prev_time = now;

        output // Return control output
    }
}

// Struct to represent a simple object with mass
struct SimpleObject {
    position: f64,  // Current position
    velocity: f64,  // Current velocity
    mass: f64,      // Mass of the object
}

impl SimpleObject {
    // Constructor for SimpleObject
    fn new(mass: f64) -> Self {
        SimpleObject {
            position: 0.0,
            velocity: 0.0,
            mass,
        }
    }

    // Function to update object state based on force applied
    fn update(&mut self, force: f64, dt: f64) {
        // Calculate acceleration using Newton's second law (F = ma)
        let acceleration = force / self.mass;
        // Update position using velocity and time step
        self.position += self.velocity * dt + 0.5* acceleration *dt * dt;
        // Update velocity using acceleration and time step
        self.velocity += acceleration * dt;
    }

    // Function to get current position of the object
    fn get_position(&self) -> f64 {
        self.position // Return current position
    }
}

fn main() {
    // Create a PID controller with specified gains and setpoint
    let mut pid_controller = PIDController::new(1.0, 0.1, 0.2, 10.0);
    // Create a simple object with specified mass
    let mut simple_object = SimpleObject::new(0.1); // Mass of the object

    // Initialize time variables
    let mut current_time = Instant::now();
    let mut last_time = current_time;

    // Main control loop
    loop {
        // Calculate time difference since last iteration
        current_time = Instant::now();
        let dt = (current_time - last_time).as_secs_f64();
        last_time = current_time;

        // Compute control output using PID controller
        let current_position = simple_object.get_position();
        let output = pid_controller.compute(current_position);

        // Apply control output to the object to update its state
        simple_object.update(output, dt);

        // Print current position of the object
        println!("dt: {:.4} Position: {:.2}", dt, simple_object.get_position());
        // Simulate delay between iterations
        std::thread::sleep(Duration::from_millis(10));

        // Check if the object has reached the setpoint
        if (simple_object.get_position() - pid_controller.setpoint).abs() < 0.003 {
            println!("Setpoint reached!");
            break; // Exit loop if setpoint reached
        }
    }
}
