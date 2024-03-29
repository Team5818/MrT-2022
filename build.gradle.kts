plugins {
    id("org.rivierarobotics.gradlerioredux") version "0.9.7"
}

gradleRioRedux {
    robotClass = "org.rivierarobotics.robot.Robot"
    teamNumber = 5818
}

repositories {
    mavenCentral()
    maven {
        name = "octyl.net"
        url = uri("https://maven.octyl.net/repository/team5818-releases")
    }
}

dependencies {
    implementation("org.rivierarobotics:5818-lib:0.3.2")
    implementation("org.rivierarobotics.apparjacktus:apparjacktus:0.1.2")
    implementation ("com.google.code.gson:gson:2.9.0")
    implementation("org.apache.commons:commons-math3:3.6.1")
}

// Gradle RIO is not applied until this is called!
gradleRioRedux.applyGradleRioConfiguration()
