plugins {
    id("org.rivierarobotics.gradlerioredux") version "0.9.2"
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
    implementation("org.rivierarobotics:5818-lib:0.2.1")
    implementation("org.rivierarobotics.apparjacktus:apparjacktus:0.1.2")

    implementation ("com.google.code.gson:gson:2.8.9")
    implementation("org.apache.commons:commons-math3:3.6.1")

    compileOnly("net.octyl.apt-creator:apt-creator-annotations:0.1.4")
    annotationProcessor("net.octyl.apt-creator:apt-creator-processor:0.1.4")
}

// Gradle RIO is not applied until this is called!
gradleRioRedux.applyGradleRioConfiguration()
