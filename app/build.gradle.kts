plugins {
    id("com.android.library")
    id("maven-publish")
}

android {
    namespace = "com.pidpilot.ftc"
    compileSdk = 36

    defaultConfig {
        minSdk = 24
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

    // Declare which built variant maven-publish is allowed to publish. Without this the
    // `release` software component does not exist, so the publication below has nothing to
    // attach and JitPack ends up shipping an empty POM with no compiled classes.
    publishing {
        singleVariant("release") {
            withSourcesJar()
        }
    }
}

dependencies {
    // compileOnly on purpose: the consuming TeamCode project already provides the FTC SDK and
    // Dashboard, so these must NOT be published as transitive dependencies (that would force a
    // possibly-conflicting version onto every team). The published AAR carries only our classes.
    compileOnly("org.firstinspires.ftc:RobotCore:11.2.0")
    compileOnly("org.firstinspires.ftc:Hardware:11.2.0")
    compileOnly("com.acmerobotics.dashboard:dashboard:0.6.0")

}

// afterEvaluate is required: AGP creates the `release` component during evaluation, so it is not
// available when this script's top-level body runs.
afterEvaluate {
    publishing {
        publications {
            create<MavenPublication>("release") {
                from(components["release"])
                groupId = "com.github.PIDPilot"
                artifactId = "ftc"
                version = "v1.0.4"
            }
        }
    }
}
