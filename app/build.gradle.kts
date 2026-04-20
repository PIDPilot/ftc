plugins {
    id("com.android.library")
    id("maven-publish")
}

android {
    namespace = "com.pidpilot.ftc"
    compileSdk = 34

    defaultConfig {
        minSdk = 24
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
}

dependencies {
    compileOnly("org.firstinspires.ftc:RobotCore:9.0.1")
    compileOnly("org.firstinspires.ftc:Hardware:9.0.1")
    compileOnly("com.acmerobotics.dashboard:dashboard:0.4.16")

}

publishing {
    publications {
        create<MavenPublication>("release") {
            groupId = "com.github.PIDPilot"
            artifactId = "ftc"
            version = "v1.0.3"
        }
    }
}