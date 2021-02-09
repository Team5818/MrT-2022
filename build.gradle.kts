import com.techshroom.inciseblue.commonLib

plugins {
    id("org.rivierarobotics.gradlerioredux") version "0.8.0"
}

gradleRioRedux {
    robotClass = "org.rivierarobotics.robot.Robot"
    teamNumber = 5818
}

repositories {
    jcenter()
    maven {
        url = uri("https://dl.bintray.com/team5818/maven-release")
    }
}

dependencies {
    implementation("org.rivierarobotics:5818-lib:0.1.0")
    implementation("org.rivierarobotics.apparjacktus:apparjacktus:0.1.1")
    commonLib("net.octyl.apt-creator", "apt-creator", "0.1.4") {
        compileOnly(lib("annotations"))
        annotationProcessor(lib("processor"))
    }
    commonLib("com.google.dagger", "dagger", "2.25.4") {
        implementation(lib())
        annotationProcessor(lib("compiler"))
    }
    simulation("edu.wpi.first.halsim:halsim_ds_socket:${wpi.wpilibVersion}:${edu.wpi.first.toolchain.NativePlatforms.desktop}@zip")
}
