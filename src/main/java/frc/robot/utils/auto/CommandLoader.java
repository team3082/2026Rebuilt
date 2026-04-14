package frc.robot.utils.auto;

import edu.wpi.first.wpilibj2.command.Command;

import java.lang.reflect.Constructor;
import java.lang.reflect.Parameter;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class CommandLoader {
    public static class ParameterInfo {
        public final String name;
        public final String type;

        public ParameterInfo(String name, String type) {
            this.name = name;
            this.type = type;
        }

        @Override
        public String toString() {
            return "{Name:" + name + ", Type:" + type + "}";
        }
    }

    public static class CommandConstructorInfo {
        public final String name;
        public final List<ParameterInfo> parameters;

        public CommandConstructorInfo(String name, List<ParameterInfo> parameters) {
            this.name = name;
            this.parameters = parameters;
        }

        public String getName() {
            return name;
        }

        public List<ParameterInfo> getParameters() {
            return parameters;
        }

        @Override
        public String toString() {
            StringBuilder builder = new StringBuilder("Constructor: " + name + ", Parameters: [");
            for (ParameterInfo param : parameters) {
                builder.append(param).append(", ");
            }
            if (!parameters.isEmpty()) {
                builder.setLength(builder.length() - 2);
            }
            builder.append("]");
            return builder.toString();
        }
    }

    // Method to load and collect command constructors with primitive parameters
    public static ArrayList<CommandConstructorInfo> loadCommandConstructors() {
        ArrayList<Class<?>> commandClasses = getCommandClasses();
        ArrayList<CommandConstructorInfo> commandConstructors = new ArrayList<>();
            
        // Loop through all provided classes
        for (Class<?> clazz : commandClasses) {
            // Iterate over constructors of each class
            for (Constructor<?> constructor : clazz.getDeclaredConstructors()) {
                // If the constructor is annotated with ChickenPlannable
                if (constructor.isAnnotationPresent(ChickenPlannable.class)) {
                    List<ParameterInfo> parameterInfos = new ArrayList<>();
                    boolean isValid = true;

                    // Check if all parameters are primitive types
                    for (Parameter parameter : constructor.getParameters()) {
                        System.out.println(parameter);
                        if (!isPrimitive(parameter.getType())) {
                            isValid = false;
                            break;
                        }
                        // Add valid primitive parameter to the list
                        parameterInfos.add(new ParameterInfo(parameter.getName(), parameter.getType().getName()));
                    }

                    // If valid (only primitive parameters), add the constructor info
                    if (isValid) {
                        CommandConstructorInfo commandConstructorInfo = new CommandConstructorInfo(
                                constructor.getName(), parameterInfos);
                        commandConstructors.add(commandConstructorInfo);
                    }
                }
            }
        }
        return commandConstructors;
    }

    // Helper method to check if a class is a primitive type or its wrapper
    private static boolean isPrimitive(Class<?> clazz) {
        return clazz.isPrimitive() || clazz == Integer.class || clazz == Long.class
                || clazz == Double.class || clazz == Float.class || clazz == Boolean.class
                || clazz == Character.class || clazz == Byte.class || clazz == Short.class;
    }

    // Recursively find all Java files in a directory and its subdirectories
    public static void findJavaFiles(File directory, ArrayList<File> javaFiles) {
        if (directory.exists() && directory.isDirectory()) {
            File[] files = directory.listFiles();
            
            if (files != null) {
                for (File file : files) {
                    if (file.isDirectory()) {
                        // Recursively search subdirectories
                        findJavaFiles(file, javaFiles);
                    } else if (file.getName().endsWith(".java")) {
                        // Add .java files to the list
                        javaFiles.add(file);
                    }
                }
            }
        }
    }

    public static ArrayList<Class<?>> getCommandClasses() {
        ArrayList<Class<?>> commandClasses = new ArrayList<>();

        File dir = new File("src/main/java/frc/robot");
        if (dir.exists() && dir.isDirectory()) {
            ArrayList<File> files = new ArrayList<>();
            findJavaFiles(dir, files);

            for (File file : files) {
                String className = getClassNameFromFile(file);
                try {
                    Class<?> clazz = Class.forName(className);
                    
                    if (Command.class.isAssignableFrom(clazz)) {
                        commandClasses.add(clazz);
                    }
                } catch (Exception e) {
                    System.out.println("Error loading class: " + className);
                    e.printStackTrace();
                }
            }
        }
        return commandClasses;
    }

    private static String getClassNameFromFile(File file) {
        ArrayList<String> directories = new ArrayList<>();

        while (file != null && !file.getName().equals("robot")) {
            directories.add(0, file.getName());
            file = file.getParentFile();
        }

        if (file == null || !file.getName().equals("robot")) {
            throw new IllegalStateException("Could not find the 'robot' directory in the path.");
        }

        StringBuilder className = new StringBuilder("frc.robot");

        for (String dir : directories) {
            className.append(".").append(dir);
        }
        
        return className.toString().replace(".java", "");
    }
}
