#include "yaml_read.hpp"

int main(int argc, char** argv)
{
    YAML_READ *yaml_read_node;
    string file_name = "/home/kay/Documents/cpp_test/yalm_study/config/test.yaml";

    yaml_read_node = new YAML_READ(file_name);
    yaml_read_node->YamlLoadFile();

    delete yaml_read_node;

    return 0;
}