#%%
import yaml
#%%
conda_yaml_file = "./conda_env.yml"
requirements_file = "./requirements.txt"

with open(conda_yaml_file, "r") as f:
    conda_yaml = yaml.safe_load(f)

dependencies = conda_yaml["dependencies"]
#%%
with open(requirements_file, "w") as f:
    for dependency in dependencies:
        if isinstance(dependency, str):
            f.write(dependency + "\n")
        elif isinstance(dependency, dict):
            # handle case where these are pip dependencies
            if "pip" in dependency:
                pip_dependencies = dependency["pip"]
                for pip_dependency in pip_dependencies:
                    f.write(pip_dependency + "\n")
            else:
                for package, version in dependency.items():
                    f.write(package + "==" + version + "\n")

# %%
