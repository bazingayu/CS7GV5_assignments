#include <windows.h>
#include <mmsystem.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <vector> 
#include <map>

// OpenGL includes
#include <GL/glew.h>
#include <GL/freeglut.h>

// GLM includes
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
glm::vec3 axis;

// Assimp includes
#include <assimp/cimport.h> 
#include <assimp/scene.h> 
#include <assimp/postprocess.h>

// font
# include <ft2build.h>
#include FT_FREETYPE_H

#include <math.h>
#define M_PI acos(-1)

FT_Library ft;
FT_Face face;

// Loading photos
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// Project includes
#include "maths_funcs.h"
double arrivalDist = 0.00005;

GLfloat rotate_x_angle = 0.0f;
GLfloat rotate_y_angle = 0.0f;
GLfloat rotate_z_angle = 0.0f;
GLfloat propeller_angle = 0.0f;
std::vector<GLfloat> angles(5, 0.0f);
float armLength = 3.9f;
using namespace std;
// control the analysis method or Cyclic Coordinate Descent
bool mode = false;

#pragma region CCD
// data structure and functions used for CCD
double simplifyAngle(double angle)
{
    while (angle >  M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle <= - M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}
// This class is used internally by the CalcIK_2D_CCD function to represent a bone in
// world space.
class Bone_2D_CCD_World {
public :
    double x;        // x position in world space
    double y;        // y position in world space
    double angle;    // angle in world space
    double cosAngle; // sine of angle
    double sinAngle; // cosine of angle
};


// This class is used to supply the CalcIK_2D_CCD function with a bone's representation
// relative to its parent in the kinematic chain.
class Bone_2D_CCD {
public:
    double angle; // angle in parent space (rad)
    double l; // bone length
    double min; // rotation limits
    double max;

    // constructers
    Bone_2D_CCD(double _l, double _min, double _max) {
        angle = 0;
        l = _l;
        min = _min;
        max = _max;
    }
    Bone_2D_CCD(double _l, double _lim) : Bone_2D_CCD(_l, -_lim, _lim) {
    }
    Bone_2D_CCD(double _l) : Bone_2D_CCD(_l, M_PI) {
    }
};

std::vector<Bone_2D_CCD> bones;

int effectors = 3;
float last_theta = 0.0f;

#pragma endregion CCD


/*----------------------------------------------------------------------------
MESH TO LOAD
----------------------------------------------------------------------------*/
#define ARM_MESH "./models/cone2.obj"
#define BALL_MESH "./models/teapot.dae"
#define BASE_MESH "./models/square.dae"
/*----------------------------------------------------------------------------
----------------------------------------------------------------------------*/

// Camera
glm::vec3 cameraPosition = glm::vec3(10.0f, 5.0f, 20.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);

glm::vec3 endPosition(19.5, 0.0f, 0.0f);
glm::vec3 links[4] = {
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(3.9f, 0.0f, 0.0f),
        glm::vec3(7.8f, 0.0f, 0.0f),
        glm::vec3(11.7f, 0.0f, 0.0f)
};
vector<double> bone_len{3.9, 3.9, 3.9, 3.9, 3.9};

#pragma region SimpleTypes
typedef struct
{
	size_t mPointCount = 0;
	std::vector<vec3> mVertices;
	std::vector<vec3> mNormals;
	std::vector<vec2> mTextureCoords;
} ModelData;
#pragma endregion SimpleTypes

using namespace std;
GLuint reflectionShaderProgramID, skyboxShaderProgramID, textShaderProgramID;

ModelData mesh_data;
int width = 800;
int height = 600;

float delta;
GLuint loc1, loc2;
GLfloat rotate_y = 0.0f;

const GLuint i = 6;
GLuint VAO[i], VBO[i * 2]; 
std::vector < ModelData > meshData;
std::vector < const char* > dataArray;

bool quan = false;


// ------------ SKYBOX ------------
unsigned int skyboxVAO, skyboxVBO;
unsigned int cubemapTexture;
vector<std::string> faces
{
	"./skybox/1x+.png",
	"./skybox/1x-.png",
	"./skybox/1y+.png",
	"./skybox/1y-.png",
	"./skybox/1z+.png",
	"./skybox/1z-.png"
};
#pragma region SKYBOX
float skyboxVertices[] = {
	-200.0f,  200.0f, -200.0f,
	-200.0f, -200.0f, -200.0f,
	 200.0f, -200.0f, -200.0f,
	 200.0f, -200.0f, -200.0f,
	 200.0f,  200.0f, -200.0f,
	-200.0f,  200.0f, -200.0f,

	-200.0f, -200.0f,  200.0f,
	-200.0f, -200.0f, -200.0f,
	-200.0f,  200.0f, -200.0f,
	-200.0f,  200.0f, -200.0f,
	-200.0f,  200.0f,  200.0f,
	-200.0f, -200.0f,  200.0f,

	 200.0f, -200.0f, -200.0f,
	 200.0f, -200.0f,  200.0f,
	 200.0f,  200.0f,  200.0f,
	 200.0f,  200.0f,  200.0f,
	 200.0f,  200.0f, -200.0f,
	 200.0f, -200.0f, -200.0f,

	-200.0f, -200.0f,  200.0f,
	-200.0f,  200.0f,  200.0f,
	 200.0f,  200.0f,  200.0f,
	 200.0f,  200.0f,  200.0f,
	 200.0f, -200.0f,  200.0f,
	-200.0f, -200.0f,  200.0f,

	-200.0f,  200.0f, -200.0f,
	 200.0f,  200.0f, -200.0f,
	 200.0f,  200.0f,  200.0f,
	 200.0f,  200.0f,  200.0f,
	-200.0f,  200.0f,  200.0f,
	-200.0f,  200.0f, -200.0f,

	-200.0f, -200.0f, -200.0f,
	-200.0f, -200.0f,  200.0f,
	 200.0f, -200.0f, -200.0f,
	 200.0f, -200.0f, -200.0f,
	-200.0f, -200.0f,  200.0f,
	 200.0f, -200.0f,  200.0f
};
#pragma endregion SKYBOX
#pragma region MESH LOADING

// ------------ FREETYPE/TEXT/FONT ------------
// This code has been taken from https://learnopengl.com/In-Practice/Text-Rendering.
#pragma region TEXT
unsigned int textVAO, textVBO;

// This struct is used to store the individual character information.
struct Character {
	unsigned int TextureID;
	glm::ivec2   Size;
	glm::ivec2   Bearing;
	unsigned int Advance;
};

std::map<char, Character> Characters;

int createFont() {
    FT_Library ft;
    if (FT_Init_FreeType(&ft))
    {
        std::cout << "Error loading FreeType Library" << std::endl;
        return -1;
    }

    FT_Face face;
    if (FT_New_Face(ft, "./fonts/arial.ttf", 0, &face))
    {
        std::cout << "Error loading Font" << std::endl;
        return -1;
    }
    else {
        FT_Set_Pixel_Sizes(face, 0, 25);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        for (unsigned char c = 0; c < 128; c++)
        {
            if (FT_Load_Char(face, c, FT_LOAD_RENDER))
            {
                std::cout << "Error loading Glyph" << std::endl;
                continue;
            }
            unsigned int texture;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(
                    GL_TEXTURE_2D,
                    0,
                    GL_RED,
                    face->glyph->bitmap.width,
                    face->glyph->bitmap.rows,
                    0,
                    GL_RED,
                    GL_UNSIGNED_BYTE,
                    face->glyph->bitmap.buffer
            );
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            Character character = {
                    texture,
                    glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
                    glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
                    face->glyph->advance.x
            };
            Characters.insert(std::pair<char, Character>(c, character));
        }
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    FT_Done_Face(face);
    FT_Done_FreeType(ft);
}

void RenderText(GLuint shaderProgramID, std::string text, float x, float y, float scale, glm::vec3 color)
{
    glUseProgram(shaderProgramID);
    glUniform3f(glGetUniformLocation(shaderProgramID, "textColor"), color.x, color.y, color.z);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(textVAO);

    std::string::const_iterator c;
    for (c = text.begin(); c != text.end(); c++)
    {
        Character ch = Characters[*c];

        float xpos = x + ch.Bearing.x * scale;
        float ypos = y - (ch.Size.y - ch.Bearing.y) * scale;

        float w = ch.Size.x * scale;
        float h = ch.Size.y * scale;
        float vertices[6][4] = {
                { xpos,     ypos + h,   0.0f, 0.0f },
                { xpos,     ypos,       0.0f, 1.0f },
                { xpos + w, ypos,       1.0f, 1.0f },

                { xpos,     ypos + h,   0.0f, 0.0f },
                { xpos + w, ypos,       1.0f, 1.0f },
                { xpos + w, ypos + h,   1.0f, 0.0f }
        };
        glBindTexture(GL_TEXTURE_2D, ch.TextureID);
        glBindBuffer(GL_ARRAY_BUFFER, textVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        x += (ch.Advance >> 6) * scale;
    }
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

/*----------------------------------------------------------------------------
MESH LOADING FUNCTION
----------------------------------------------------------------------------*/

ModelData load_mesh(const char* file_name) {
	ModelData modelData;

	/* Use assimp to read the model file, forcing it to be read as    */
	/* triangles. The second flag (aiProcess_PreTransformVertices) is */
	/* relevant if there are multiple meshes in the model file that   */
	/* are offset from the origin. This is pre-transform them so      */
	/* they're in the right position.                                 */
	const aiScene* scene = aiImportFile(
		file_name, 
		aiProcess_Triangulate | aiProcess_PreTransformVertices
	); 

	if (!scene) {
		fprintf(stderr, "ERROR: reading mesh %s\n", file_name);
		return modelData;
	}

	printf("  %i materials\n", scene->mNumMaterials);
	printf("  %i meshes\n", scene->mNumMeshes);
	printf("  %i textures\n", scene->mNumTextures);

	for (unsigned int m_i = 0; m_i < scene->mNumMeshes; m_i++) {
		const aiMesh* mesh = scene->mMeshes[m_i];
		printf("    %i vertices in mesh\n", mesh->mNumVertices);
		modelData.mPointCount += mesh->mNumVertices;
		for (unsigned int v_i = 0; v_i < mesh->mNumVertices; v_i++) {
			if (mesh->HasPositions()) {
				const aiVector3D* vp = &(mesh->mVertices[v_i]);
				modelData.mVertices.push_back(vec3(vp->x, vp->y, vp->z));
			}
			if (mesh->HasNormals()) {
				const aiVector3D* vn = &(mesh->mNormals[v_i]);
				modelData.mNormals.push_back(vec3(vn->x, vn->y, vn->z));
			}
			if (mesh->HasTextureCoords(0)) {
				const aiVector3D* vt = &(mesh->mTextureCoords[0][v_i]);
				modelData.mTextureCoords.push_back(vec2(vt->x, vt->y));
			}
			if (mesh->HasTangentsAndBitangents()) {
				/* You can extract tangents and bitangents here              */
				/* Note that you might need to make Assimp generate this     */
				/* data for you. Take a look at the flags that aiImportFile  */
				/* can take.                                                 */
			}
		}
	}

	aiReleaseImport(scene);
	return modelData;
}

#pragma endregion MESH LOADING

// Shader Functions- click on + to expand
#pragma region SHADER_FUNCTIONS
char* readShaderSource(const char* shaderFile) {
	FILE* fp;
	fopen_s(&fp, shaderFile, "rb");

	if (fp == NULL) { return NULL; }

	fseek(fp, 0L, SEEK_END);
	long size = ftell(fp);

	fseek(fp, 0L, SEEK_SET);
	char* buf = new char[size + 1];
	fread(buf, 1, size, fp);
	buf[size] = '\0';

	fclose(fp);

	return buf;
}


static void AddShader(GLuint ShaderProgram, const char* pShaderText, GLenum ShaderType)
{
	// create a shader object
	GLuint ShaderObj = glCreateShader(ShaderType);

	if (ShaderObj == 0) {
		std::cerr << "Error creating shader..." << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}
	const char* pShaderSource = readShaderSource(pShaderText);

	// Bind the source code to the shader, this happens before compilation
	glShaderSource(ShaderObj, 1, (const GLchar**)&pShaderSource, NULL);
	// compile the shader and check for errors
	glCompileShader(ShaderObj);
	GLint success;
	// check for shader related errors using glGetShaderiv
	glGetShaderiv(ShaderObj, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar InfoLog[1024] = { '\0' };
		glGetShaderInfoLog(ShaderObj, 1024, NULL, InfoLog);
		std::cerr << "Error compiling "
			<< (ShaderType == GL_VERTEX_SHADER ? "vertex" : "fragment")
			<< " shader program: " << InfoLog << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}
	// Attach the compiled shader object to the program object
	glAttachShader(ShaderProgram, ShaderObj);
}

GLuint CompileShaders(const char* vertexShader, const char* fragmentShader)
{
	//Start the process of setting up our shaders by creating a program ID
	//Note: we will link all the shaders together into this ID
	GLuint shaderProgramID = glCreateProgram();
	if (shaderProgramID == 0) {
		std::cerr << "Error creating shader program..." << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}

	// Create two shader objects, one for the vertex, and one for the fragment shader
	AddShader(shaderProgramID, vertexShader, GL_VERTEX_SHADER);
	AddShader(shaderProgramID, fragmentShader, GL_FRAGMENT_SHADER);

	GLint Success = 0;
	GLchar ErrorLog[1024] = { '\0' };
	// After compiling all shader objects and attaching them to the program, we can finally link it
	glLinkProgram(shaderProgramID);
	// check for program related errors using glGetProgramiv
	glGetProgramiv(shaderProgramID, GL_LINK_STATUS, &Success);
	if (Success == 0) {
		glGetProgramInfoLog(shaderProgramID, sizeof(ErrorLog), NULL, ErrorLog);
		std::cerr << "Error linking shader program: " << ErrorLog << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}

	// program has been successfully linked but needs to be validated to check whether the program can execute given the current pipeline state
	glValidateProgram(shaderProgramID);
	// check for program related errors using glGetProgramiv
	glGetProgramiv(shaderProgramID, GL_VALIDATE_STATUS, &Success);
	if (!Success) {
		glGetProgramInfoLog(shaderProgramID, sizeof(ErrorLog), NULL, ErrorLog);
		std::cerr << "Invalid shader program: " << ErrorLog << std::endl;
		std::cerr << "Press enter/return to exit..." << std::endl;
		std::cin.get();
		exit(1);
	}
	// Finally, use the linked shader program
	// Note: this program will stay in effect for all draw calls until you replace it with another or explicitly disable its use
	glUseProgram(shaderProgramID);
	return shaderProgramID;
}
#pragma endregion SHADER_FUNCTIONS

GLuint loadCubemap(vector<std::string> faces)
{
	GLuint skyboxTextureID;
	glGenTextures(1, &skyboxTextureID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTextureID);

	int width, height, nrChannels;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		stbi_image_free(data);
	}

	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	return skyboxTextureID;
}
// VBO Functions - click on + to expand
#pragma region VBO_FUNCTIONS
void generateObjectBufferMesh(std::vector < const char* > dataArray) {
	/*----------------------------------------------------------------------------
	LOAD MESH HERE AND COPY INTO BUFFERS
	----------------------------------------------------------------------------*/

	loc1 = glGetAttribLocation(reflectionShaderProgramID, "vertex_position");
	loc2 = glGetAttribLocation(reflectionShaderProgramID, "vertex_normals");
	//loc3 = glGetAttribLocation(shaderProgramID, "vertex_texture");
	int counter = 0;
	for (int i = 0; i < dataArray.size(); i++) {
		mesh_data = load_mesh(dataArray[i]);
		meshData.push_back(mesh_data);

		glGenBuffers(1, &VBO[counter]);
		glBindBuffer(GL_ARRAY_BUFFER, VBO[counter]);
		glBufferData(GL_ARRAY_BUFFER, mesh_data.mPointCount * sizeof(vec3), &mesh_data.mVertices[0], GL_STATIC_DRAW);

		glGenBuffers(1, &VBO[counter + 1]);
		glBindBuffer(GL_ARRAY_BUFFER, VBO[counter + 1]);
		glBufferData(GL_ARRAY_BUFFER, mesh_data.mPointCount * sizeof(vec3), &mesh_data.mNormals[0], GL_STATIC_DRAW);

		glGenVertexArrays(1, &VAO[i]);
		glBindVertexArray(VAO[i]);

		glEnableVertexAttribArray(loc1);
		glBindBuffer(GL_ARRAY_BUFFER, VBO[counter]);
		glVertexAttribPointer(loc1, 3, GL_FLOAT, GL_FALSE, 0, NULL);

		glEnableVertexAttribArray(loc2);
		glBindBuffer(GL_ARRAY_BUFFER, VBO[counter + 1]);
		glVertexAttribPointer(loc2, 3, GL_FLOAT, GL_FALSE, 0, NULL);

		counter += 2;
	}
}

void Initialize_Bones(vector<double> l){
    for(int ii = 0; ii < l.size(); ii ++){
        Bone_2D_CCD bone((l[ii]));
        bones.push_back(bone);
    }
}

void generateFontBufferObjects() {
    glGenVertexArrays(1, &textVAO);
    glGenBuffers(1, &textVBO);
    glBindVertexArray(textVAO);
    glBindBuffer(GL_ARRAY_BUFFER, textVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void generateSkybox() {
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
}
#pragma endregion VBO_FUNCTIONS


bool computerIK(){
    float l1 = armLength, l2 = armLength; //bone length
    float ex = endPosition.x , ey = endPosition.y; // adjusted target position
    float sin2=0.0f, cos2=0.0f, tan1=0.0f; //sin and cos of angle 2
    float angle1=0.0f, angle2=0.0f; //angle 1 and 2 in radians;

    cos2 = ((ex*ex) + (ey*ey) - (l1*l1) -(l2*l2)) / (2*l1*l2);
    if(cos2>=-1.0 && cos2<=1.0){
        cos2 = max(-1.0f, min(1.0f, cos2));
        angle2 = (float) acos(cos2);

//        cout << angle2 << endl;
        angles[1] = glm::degrees(angle2);

        sin2 = (float)glm::sin(angle2);


//        tan1 = (-(l2 * sin2 * ex) + ((l1 + (l2 * cos2)) * ey)) / ((l2 * sin2 * ey) + ((l1 + (l2 * cos2)) * ex));
//        angles[0] = glm::degrees(glm::atan(tan1));

        float triAdjacent = l1 + l2*cos2;
        float triOpposite = l2 * sin2;

        float tanY = ey*triAdjacent - ex*triOpposite;
        float tanX = ex*triAdjacent + ey*triOpposite;

        angles[0] = glm::degrees(glm::atan(tanY/tanX));
        return true;

    }
    else{
        angles[0] = last_theta;
        angles[1] = angles[0] - glm::radians(150.0f);
        cout << "can't reach that point" << endl;
    }
    last_theta = angles[0];
}


//refer to https://www.ryanjuckett.com/cyclic-coordinate-descent-in-2d/
std::string CalcIK_2D_CCD(double targetX, double targetY) {
    // Given a bone chain located at the origin, this function will perform a single cyclic
    // coordinate descent (CCD) iteration. This finds a solution of bone angles that places
    // the final bone in the given chain at a target position. The supplied bone angles are
    // used to prime the CCD iteration. If a valid solution does not exist, the angles will
    // move as close to the target as possible. The user should resupply the updated angles
    // until a valid solution is found (or until an iteration limit is met).
    //
    // returns: Success when a valid solution was found.
    //          Processing when still searching for a valid solution.
    //          Failure when it can get no closer to the target.
    // Set an epsilon value to prevent division by small numbers.
    const double epsilon = 0.0001;

    // Set max arc length a bone can move the end effector an be considered no motion
    // so that we can detect a failure state.
    const double trivialArcLength = 0.00001;

    // how many bones we have
    int numBones = bones.size();
    assert(numBones > 0);

    // closest to the target point
    double arrivalDistSqr = arrivalDist * arrivalDist;

    //===
    // Generate the world space bone data.
    std::vector<Bone_2D_CCD_World> worldBones;

    // Start with the root bone.
    Bone_2D_CCD_World rootWorldBone;
    rootWorldBone.angle = bones[0].angle;
    rootWorldBone.x = bones[0].l * cos(rootWorldBone.angle);
    rootWorldBone.y = bones[0].l * sin(rootWorldBone.angle);

    rootWorldBone.cosAngle = cos(rootWorldBone.angle);
    rootWorldBone.sinAngle = sin(rootWorldBone.angle);
    worldBones.push_back(rootWorldBone);

    // Convert All bones to world space.
    for (int boneIdx = 1; boneIdx < numBones; boneIdx++)
    {
        Bone_2D_CCD_World prevWorldBone = worldBones[boneIdx - 1];
        Bone_2D_CCD curLocalBone = bones[boneIdx];

        Bone_2D_CCD_World newWorldBone;
        newWorldBone.angle = prevWorldBone.angle + curLocalBone.angle;
        newWorldBone.cosAngle = cos(newWorldBone.angle);
        newWorldBone.sinAngle = sin(newWorldBone.angle);
        newWorldBone.x = prevWorldBone.x + newWorldBone.cosAngle * curLocalBone.l;
        newWorldBone.y = prevWorldBone.y + newWorldBone.sinAngle * curLocalBone.l;
        worldBones.push_back(newWorldBone);
    }

    //===
    // Track the end effector position (the final bone)
    double endX = worldBones[numBones - 1].x;
    double endY = worldBones[numBones - 1].y;

    //=== start calculation
    // Perform CCD on the bones by optimizing each bone in a loop
    // from the final bone to the root bone
    bool modifiedBones = false;
    for (int boneIdx = numBones -1; boneIdx > 0; boneIdx--)
    {   cout << "bone " << boneIdx << endl;
        // Get the vector from the current bone to the end effector position.
        double curToEndX = endX - worldBones[boneIdx-1].x;
        double curToEndY = endY - worldBones[boneIdx-1].y;
        double curToEndMag = sqrt(curToEndX * curToEndX + curToEndY * curToEndY);

        // Get the vector from the current bone to the target position.
        double curToTargetX = targetX - worldBones[boneIdx-1].x;
        double curToTargetY = targetY - worldBones[boneIdx-1].y;
        double curToTargetMag = sqrt(curToTargetX * curToTargetX + curToTargetY * curToTargetY);

        // Get rotation to place the end effector on the line from the current
        // joint position to the target postion.
        double cosRotAng;
        double sinRotAng;
        double endTargetMag = curToEndMag * curToTargetMag;
        if (endTargetMag <= epsilon){
            cosRotAng = 1;
            sinRotAng = 0;
        }
        else {
            cosRotAng = (curToEndX * curToTargetX + curToEndY * curToTargetY) / endTargetMag;
            sinRotAng = (curToEndX * curToTargetY - curToEndY * curToTargetX) / endTargetMag;
        }

        // Clamp the cosine into range when computing the angle (might be out of range
        // due to floating point error).
        double rotAng = acos(max(-1.0, min(1.0, cosRotAng)));
        // correct angle value regarding its sine
        if (sinRotAng < 0.0)
            rotAng = -rotAng;
        // apply rotation limits
        double M = bones[boneIdx].max;
        double m = bones[boneIdx].min;
        double newAngle = simplifyAngle(bones[boneIdx].angle + rotAng);
//        cout << newAngle << " " << bones[boneIdx].angle << " " << rotAng << endl;
        if (newAngle > M || newAngle < m )
            rotAng = simplifyAngle(newAngle - M) < simplifyAngle(newAngle - m) ? M-bones[boneIdx].angle : m-bones[boneIdx].angle;
        cosRotAng = cos(rotAng);
        sinRotAng = sin(rotAng);

        // Rotate the end effector position
        endX = worldBones[boneIdx-1].x + cosRotAng * curToEndX - sinRotAng * curToEndY;
        endY = worldBones[boneIdx-1].y + sinRotAng * curToEndX + cosRotAng * curToEndY;

//        cout << endX << " " << endY << " " << endPosition.x << " " << endPosition.y << endl << rotAng;

        // Rotate the current bone in local space
        bones[boneIdx].angle = simplifyAngle(bones[boneIdx].angle + rotAng);

        // Check for termination
        double endToTargetX = (targetX - endX);
        double endToTargetY = (targetY - endY);

        if (endToTargetX * endToTargetX + endToTargetY * endToTargetY <= arrivalDistSqr) {
            // We found a valid solution.
            return "success";
        }

        // Track if the arc length that we moved the end effector was
        // a nontrivial distance.
        if (!modifiedBones && abs(rotAng) * curToEndMag > trivialArcLength) {
            modifiedBones = true;
        }
    }

    // We have to handle the first bone (index 0) separately
    // because the algorithm needs value for the previous bone

    // Get the vector from the current bone to the end effector position.
    double curToEndX = endX;
    double curToEndY = endY;
    double curToEndMag = sqrt(curToEndX * curToEndX + curToEndY * curToEndY);

    // Get the vector from the current bone to the target position.
    double curToTargetX = targetX;
    double curToTargetY = targetY;
    double curToTargetMag = sqrt(curToTargetX * curToTargetX + curToTargetY * curToTargetY);

    // Get rotation to place the end effector on the line from the current
    // joint position to the target postion.
    double cosRotAng;
    double sinRotAng;
    double endTargetMag = curToEndMag * curToTargetMag;
    if (endTargetMag <= epsilon) {
        cosRotAng = 1;
        sinRotAng = 0;
    }
    else {
        cosRotAng = (curToEndX * curToTargetX + curToEndY * curToTargetY) / endTargetMag;
        sinRotAng = (curToEndX * curToTargetY - curToEndY * curToTargetX) / endTargetMag;
    }

    // Clamp the cosine into range when computing the angle (might be out of range
    // due to floating point error).
    double rotAng = acos(max(-1.0, min(1.0, cosRotAng)));
    // correct angle value regarding its sine
    if (sinRotAng < 0.0)
        rotAng = -rotAng;
    // apply rotation limits
    double M = bones[0].max;
    double m = bones[0].min;
    double newAngle = simplifyAngle(bones[0].angle + rotAng);
    if (newAngle > M || newAngle < m)
        rotAng = simplifyAngle(newAngle - M) < simplifyAngle(newAngle - m) ? M - bones[0].angle : m - bones[0].angle;
    cosRotAng = cos(rotAng);
    sinRotAng = sin(rotAng);

    // Rotate the end effector position.
    endX = cosRotAng * curToEndX - sinRotAng * curToEndY;
    endY = sinRotAng * curToEndX + cosRotAng * curToEndY;

    // Rotate the current bone in local space (this value is output to the user)
    bones[0].angle = simplifyAngle(bones[0].angle + rotAng);

    // Check for termination
    double endToTargetX = (targetX - endX);
    double endToTargetY = (targetY - endY);

    if (endToTargetX * endToTargetX + endToTargetY * endToTargetY <= arrivalDistSqr) {
        // We found a valid solution.
        return "success";
    }

    // Track if the arc length that we moved the end effector was
    // a nontrivial distance.
    if (!modifiedBones && abs(rotAng) * curToEndMag > trivialArcLength) {
        modifiedBones = true;
    }

    // We failed to find a valid solution during this iteration.
    if (modifiedBones)
        return "processing";
    else
        return "failure";
}

void move(float _x, float _y) {
    while (CalcIK_2D_CCD(_x, _y) == "processing") {
    }
    cout << CalcIK_2D_CCD(_x, _y) << endl;
    angles[0] = bones[0].angle;
    angles[1] = bones[1].angle;
    angles[2] = bones[2].angle;
    angles[3] = bones[3].angle;
    angles[4] = bones[4].angle;
}


void display() {

	glEnable(GL_DEPTH_TEST); 
	glDepthFunc(GL_LESS); 
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// View and Projection
	glm::mat4 view = glm::mat4(0.1f);
    view = glm::lookAt(cameraPosition, cameraPosition + cameraFront, cameraUp);
	glm::mat4 proj = glm::perspective(glm::radians(45.0f), (float)width / (float)height, 0.1f, 1000.0f);

	glDepthFunc(GL_LEQUAL);
	glUseProgram(skyboxShaderProgramID);
    glUniformMatrix4fv(glGetUniformLocation(skyboxShaderProgramID, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(skyboxShaderProgramID, "proj"), 1, GL_FALSE, glm::value_ptr(proj));
	glBindVertexArray(skyboxVAO);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
	glDrawArrays(GL_TRIANGLES, 0, 36);

    glEnable(GL_DEPTH_TEST); // enable depth-testing
    glMatrixMode(GL_PROJECTION);
    glDepthFunc(GL_LESS); // depth-testing interprets a smaller value as "closer"


    if (!mode){
        glm::mat4 first = glm::mat4(1.0f);
        glUseProgram(reflectionShaderProgramID);
        glBindVertexArray(VAO[0]);
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "proj"), 1, GL_FALSE, glm::value_ptr(proj));
        first = glm::rotate(first, angles[0], glm::vec3(0.0, 0.0f, 1.0f));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE, glm::value_ptr(first));
        glDrawArrays(GL_TRIANGLES, 0, meshData[0].mPointCount);

        glm::mat4 second = glm::mat4(1.0f);
        second = glm::translate(second, glm::vec3(3.9f, 0.0f, 0.0f));
        second = glm::rotate(second, angles[1], glm::vec3(0.0f, 0.0f, 1.0f));
        second = first * second;
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE,glm::value_ptr(second));
        glDrawArrays(GL_TRIANGLES, 0, meshData[0].mPointCount);

        glm::mat4 third = glm::mat4(1.0f);
        third = glm::translate(third, glm::vec3(3.9f, 0.0f, 0.0f));
        third = glm::rotate(third, angles[2], glm::vec3(0.0f, 0.0f, 1.0f));
        third = second * third;
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE,glm::value_ptr(third));
        glDrawArrays(GL_TRIANGLES, 0, meshData[0].mPointCount);

        glm::mat4 fourth = glm::mat4(1.0f);
        fourth = glm::translate(fourth, glm::vec3(3.9f, 0.0f, 0.0f));
        fourth = glm::rotate(fourth, angles[3], glm::vec3(0.0f, 0.0f, 1.0f));
        fourth = third * fourth;
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE,glm::value_ptr(fourth));
        glDrawArrays(GL_TRIANGLES, 0, meshData[0].mPointCount);

        glm::mat4 fifth = glm::mat4(1.0f);
        fifth = glm::translate(fifth, glm::vec3(3.9f, 0.0f, 0.0f));
        fifth = glm::rotate(fifth, angles[4], glm::vec3(0.0f, 0.0f, 1.0f));
        fifth = fourth * fifth;
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE,glm::value_ptr(fifth));
        glDrawArrays(GL_TRIANGLES, 0, meshData[0].mPointCount);

        glm::mat4 sixth = glm::mat4(1.0f);
        sixth = glm::translate(sixth, glm::vec3(3.9f, 0.0f, 0.0f));
        sixth = glm::rotate(sixth, angles[4]-0.3f, glm::vec3(0.0f, 0.0f, 1.0f));
        sixth = fourth * sixth;
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE,glm::value_ptr(sixth));
        glDrawArrays(GL_TRIANGLES, 0, meshData[0].mPointCount);

    }
    else{
        glm::mat4 first = glm::mat4(1.0f);
        glUseProgram(reflectionShaderProgramID);
        glBindVertexArray(VAO[0]);
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "proj"), 1, GL_FALSE, glm::value_ptr(proj));
        first = glm::rotate(first, glm::radians(angles[0]), glm::vec3(0.0, 0.0f, 1.0f));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE, glm::value_ptr(first));
        glDrawArrays(GL_TRIANGLES, 0, meshData[0].mPointCount);

        glm::mat4 second = glm::mat4(1.0f);
        second = glm::translate(second, glm::vec3(3.9f, 0.0f, 0.0f));
        second = glm::rotate(second, glm::radians(angles[1]), glm::vec3(0.0f, 0.0f, 1.0f));
        second = first * second;
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE,glm::value_ptr(second));
        glDrawArrays(GL_TRIANGLES, 0, meshData[0].mPointCount);

    }


    glm::vec3 ballColor = glm::vec3(1.0f, 0.0f, 0.0f);
    glm::mat4 ballModel = glm::mat4(1.0f);
    ballModel = glm::translate(ballModel, endPosition);
    glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE, glm::value_ptr(ballModel));
    glBindVertexArray(VAO[1]);
    glDrawArrays(GL_TRIANGLES, 0, meshData[1].mPointCount);

    glm::mat4 baseModel = glm::mat4(1.0f);
//    baseModel = glm::translate(baseModel, endPosition);
    baseModel = glm::scale(baseModel, glm::vec3(1.0f, 3.0f, 1.0f));
    glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE, glm::value_ptr(baseModel));
    glBindVertexArray(VAO[2]);
    glDrawArrays(GL_TRIANGLES, 0, meshData[2].mPointCount);




    glutSwapBuffers();
}


void updateScene() {

	static DWORD last_time = 0;
	DWORD curr_time = timeGetTime();
	if (last_time == 0)
		last_time = curr_time;
	delta = (curr_time - last_time) * 0.001f;
	last_time = curr_time;

    propeller_angle += 5.0f;

	// Rotate the model slowly around the y axis at 20 degrees per second
	rotate_y += 25.0f * delta;
	rotate_y = fmodf(rotate_y, 360.0f);

	// Draw the next frame
	glutPostRedisplay();
}


void init()
{
    Initialize_Bones(bone_len);
    // Set up the shaders
	reflectionShaderProgramID = CompileShaders("./shaders/simpleVertexShader.txt", "./shaders/phongFragmentShader.txt");
	skyboxShaderProgramID = CompileShaders("./shaders/skyboxVertexShader.txt", "./shaders/skyboxFragmentShader.txt");
	textShaderProgramID = CompileShaders("./shaders/textVertexShader.txt", "./shaders/textFragmentShader.txt");
	// Skybox
	generateSkybox();
	cubemapTexture = loadCubemap(faces);
    //fonts
    createFont();
    generateFontBufferObjects();
	// load mesh into a vertex buffer array
	dataArray.push_back(ARM_MESH);
    dataArray.push_back(BALL_MESH);
    dataArray.push_back(BASE_MESH);
    generateObjectBufferMesh(dataArray);

}

// Placeholder code for the keypress
void keypress(unsigned char key, int x, int y) {
	switch (key) {
	case '1':
		angles[0] += M_PI/36.0;
        break;
	case '2':
		angles[1] += M_PI/36.0;
		break;
	case '3':
		angles[2] += M_PI/36.0;
		break;
    case '4':
        angles[3] += M_PI/36.0;
        break;
    case '5':
        angles[4] += M_PI/36.0;
        break;
    case '7':
        angles[0] -= M_PI/36.0;
        break;
    case '8':
        angles[1] -= M_PI/36.0;
        break;
    case '9':
        angles[2] -= M_PI/36.0;
        break;
    case '0':
        angles[3] -= M_PI/36.0;
        break;
    case '-':
        angles[4] -= M_PI/36.0;
        break;
    case ' ':
        mode = !mode;
        if(!mode){
            endPosition = glm::vec3(19.5, 0.0f, 0.0f);
        }
        else{
            endPosition = glm::vec3(7.8, 0.0f, 0.0f);
        }
        break;
	}
}

void specialKeys(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_UP:
            endPosition.y += 0.1f;
            if(!mode){
                move((float)endPosition.x, (float)endPosition.y);
            }
            else computerIK();
            break;
        case GLUT_KEY_DOWN:
            endPosition.y -= 0.1f;
            if(!mode){
                move((float)endPosition.x, (float)endPosition.y);
            }
            else computerIK();
            break;
        case GLUT_KEY_RIGHT:
            endPosition.x += 0.1f;
            if(!mode){
                move((float)endPosition.x, (float)endPosition.y);
            }
            else computerIK();
            break;
        case GLUT_KEY_LEFT:
            endPosition.x -= 0.1f;
            if(!mode){
                move((float)endPosition.x, (float)endPosition.y);
            }
            else computerIK();
            break;
    }
}


int main(int argc, char** argv) {

	// Set up the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(width, height);
	glutCreateWindow("Lab 2");

	// Tell glut where the display function is
	glutDisplayFunc(display);
	glutIdleFunc(updateScene);
	glutKeyboardFunc(keypress);
    glutSpecialFunc(specialKeys);

	// A call to glewInit() must be done after glut is initialized!
	GLenum res = glewInit();
	// Check for any errors
	if (res != GLEW_OK) {
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
		return 1;
	}
	// Set up your objects and shaders
	init();
	// Begin infinite event loop
	glutMainLoop();
	return 0;
}
