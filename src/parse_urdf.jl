function parse_geometries(xml_geometry::XMLElement, package_path, file_path="")
    geometries = Union{AbstractGeometry, AbstractMesh}[]
    for xml_cylinder in get_elements_by_tagname(xml_geometry, "cylinder")
        length = rbd.parse_scalar(Float32, xml_cylinder, "length")
        radius = rbd.parse_scalar(Float32, xml_cylinder, "radius")
        push!(geometries, HyperCylinder{3, Float32}(length, radius))
    end
    for xml_box in get_elements_by_tagname(xml_geometry, "box")
        size = Vec{3, Float32}(rbd.parse_vector(Float32, xml_box, "size", "0 0 0"))
        push!(geometries, HyperRectangle(-size / 2, size))
    end
    for xml_sphere in get_elements_by_tagname(xml_geometry, "sphere")
        radius = rbd.parse_scalar(Float32, xml_sphere, "radius")
        push!(geometries, HyperSphere(zero(Point{3, Float32}), radius))
    end
    for xml_mesh in get_elements_by_tagname(xml_geometry, "mesh")
        filename = attribute(xml_mesh, "filename")
        dae_pattern = r".dae$"
        replaced_extension_with_obj = false
        if ismatch(dae_pattern, filename)
            filename = replace(filename, dae_pattern, ".obj")
            replaced_extension_with_obj = true
        end
        package_pattern = r"^package://"

        if ismatch(package_pattern, filename)
            found_mesh = false
            for package_directory in package_path
                filename_in_package = joinpath(package_directory, replace(filename, package_pattern, ""))
                if ispath(filename_in_package)
                    mesh = load(filename_in_package, GLUVMesh)
                    push!(geometries, mesh)
                    found_mesh = true
                    break
                end
            end
            if !found_mesh
                warning_message = "Could not find the mesh file: $(filename). I tried substituting the following folders for the 'package://' prefix: $(package_path)."
                if replaced_extension_with_obj
                    warning_message *= " Note that I replaced the file's original extension with .obj to try to find a mesh in a format I can actually load."
                end
                warn(warning_message)
            end
        else
            filename = joinpath(file_path, filename)
            if ispath(filename)
                mesh = load(filename)
                push!(geometries, mesh)
            else
                warning_message = "Could not find the mesh file: $(filename)."
                if replaced_extension_with_obj
                    warning_message *= " Note that I replaced the file's original extension with .obj to try to find a mesh in a format I can actually load."
                end
                warn(warning_message)
            end
        end
    end
    geometries
end

function parse_material!(materials::Dict{String, <:AbstractMaterial}, xml_material)
    if xml_material === nothing
        return MeshLambertMaterial()
    end
    name = attribute(xml_material, "name")
    material = get!(materials, name) do
        MeshLambertMaterial()
    end
    xml_color = find_element(xml_material, "color")
    if xml_color !== nothing
        default = "0.7 0.7 0.7 1."
        material.color = RGBA{Float32}(rbd.parse_vector(Float32, xml_color, "rgba", default)...)
    end
    material
end

function parse_link!(materials::Dict, xml_link,
                     package_path=ros_package_path(), file_path="", tag="visual")
    xml_visuals = get_elements_by_tagname(xml_link, tag)
    visuals = Vector{Pair{Mesh, Transformation}}
    visual_groups = map(xml_visuals) do xml_visual
        xml_material = find_element(xml_visual, tag)
        material = parse_material!(materials, find_element(xml_visual, "material"))
        rot, trans = rbd.parse_pose(Float64, find_element(xml_visual, "origin"))
        tform = AffineMap(rot, trans)
        map(parse_geometries(find_element(xml_visual, "geometry"), package_path, file_path)) do geometry
            Mesh(geometry, material) => tform
        end
    end
    reduce(vcat, [], visual_groups)
end

function create_graph(xml_links, xml_joints)
    # create graph structure of XML elements
    graph = DirectedGraph{Vertex{XMLElement}, Edge{XMLElement}}()
    vertices = Vertex.(xml_links)
    for vertex in vertices
        add_vertex!(graph, vertex)
    end
    name_to_vertex = Dict(attribute(data(v), "name") => v for v in vertices)
    for xml_joint in xml_joints
        parent = name_to_vertex[attribute(find_element(xml_joint, "parent"), "link")]
        child = name_to_vertex[attribute(find_element(xml_joint, "child"), "link")]
        add_edge!(graph, parent, child, Edge(xml_joint))
    end
    graph
end

ros_package_path() = split(get(ENV, "ROS_PACKAGE_PATH", ""), ':')

function parse_urdf_visuals(filename, mechanism;
            package_path=ros_package_path(), file_path="", tag="visual")
    xdoc = parse_file(filename)
    xroot = LightXML.root(xdoc)
    @assert LightXML.name(xroot) == "robot"

    xml_links = get_elements_by_tagname(xroot, "link")
    xml_joints = get_elements_by_tagname(xroot, "joint")
    xml_materials = get_elements_by_tagname(xroot, "material")

    graph = create_graph(xml_links, xml_joints)
    roots = collect(filter(v -> isempty(in_edges(v, graph)), rbd.Graphs.vertices(graph)))
    length(roots) != 1 && error("Can only handle a single root")
    tree = SpanningTree(graph, first(roots))

    materials = Dict{String, MeshMaterial}()
    for xml_material in xml_materials
        parse_material!(materials, xml_material)
    end

#     name_to_body = Dict(string(body) => body for body in bodies(mechanism))
    name_to_frame_and_body = Dict(string(tf.from) => (tf.from, body) for body in bodies(mechanism) for tf in rbd.frame_definitions(body))

    visuals = Dict(
        map(rbd.Graphs.vertices(tree)) do vertex
            xml_link = data(vertex)

            linkname = attribute(xml_link, "name")
            framename = if vertex == rbd.Graphs.root(tree)
                linkname
            else
                xml_joint = data(edge_to_parent(vertex, tree))
                jointname = attribute(xml_joint, "name")
                string("after_", jointname) # TODO: create function in RBD, call it here
            end
            body_frame, body = name_to_frame_and_body[framename]

            vis = parse_link!(materials, xml_link, package_path, file_path, tag)
            body_frame => vis
        end
    )
end
