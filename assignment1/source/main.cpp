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

FT_Library ft;
FT_Face face;

// Loading photos
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// Project includes
#include "maths_funcs.h"

GLfloat rotate_x_angle = 0.0f;
GLfloat rotate_y_angle = 0.0f;
GLfloat rotate_z_angle = 0.0f;
GLfloat propeller_angle = 0.0f;


/*----------------------------------------------------------------------------
MESH TO LOAD
----------------------------------------------------------------------------*/
#define SQUARE_MESH "./models/airplane.obj"
#define BACKPACK_MESH "./models/airplane_part.obj"
#define TEAPOT_MESH "./models/airplane.obj"
/*----------------------------------------------------------------------------
----------------------------------------------------------------------------*/

// Camera
glm::vec3 cameraPosition = glm::vec3(0.0f, 0.0f, 12.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);

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
    glm::mat4 model = glm::mat4(1.0f);
    if(!quan) {
        glUseProgram(reflectionShaderProgramID);
        glBindVertexArray(VAO[2]);
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "proj"), 1, GL_FALSE, glm::value_ptr(proj));
        model = glm::eulerAngleYXZ(glm::radians(rotate_y_angle), glm::radians(rotate_x_angle),
                                   glm::radians(rotate_z_angle));
        model = glm::scale(model, glm::vec3(0.5f, 0.5f, 0.5f));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE,
                           glm::value_ptr(model));
        glDrawArrays(GL_TRIANGLES, 0, meshData[2].mPointCount);
        // hierarchy
        glBindVertexArray(VAO[1]);
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "proj"), 1, GL_FALSE, glm::value_ptr(proj));
        glm::mat4 model_propeller = glm::mat4(1.0f);
        model_propeller = glm::rotate(model_propeller, glm::radians(propeller_angle), glm::vec3(0.0f, 0.0f, 1.0f));
        model_propeller = model * model_propeller;
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE,glm::value_ptr(model_propeller));
        glDrawArrays(GL_TRIANGLES, 0, meshData[1].mPointCount);
    }
    else{
        glm::mat4 model = glm::mat4(1.0f);
        glUseProgram(reflectionShaderProgramID);
        glBindVertexArray(VAO[2]);
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "proj"), 1, GL_FALSE, glm::value_ptr(proj));
        glm::qua<float> q = glm::qua<float>(glm::radians(glm::vec3(rotate_x_angle, rotate_y_angle, rotate_z_angle))); //创建一个绕z轴旋转90度的四元数
        model = glm::mat4_cast(q) * model;
        model = glm::scale(model, glm::vec3(0.5f, 0.5f, 0.5f));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glDrawArrays(GL_TRIANGLES, 0, meshData[2].mPointCount);
        // hierarchy
        glBindVertexArray(VAO[1]);
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "proj"), 1, GL_FALSE, glm::value_ptr(proj));
        glm::mat4 model_propeller = glm::mat4(1.0f);
        model_propeller = glm::rotate(model_propeller, glm::radians(propeller_angle), glm::vec3(0.0f, 0.0f, 1.0f));
        model_propeller = model * model_propeller;
        glUniformMatrix4fv(glGetUniformLocation(reflectionShaderProgramID, "model"), 1, GL_FALSE,glm::value_ptr(model_propeller));
        glDrawArrays(GL_TRIANGLES, 0, meshData[1].mPointCount);
    }




    //text
    if(!quan){
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glm::mat4 projection = glm::ortho(0.0f, 800.0f, 0.0f, 600.0f);
        glUseProgram(textShaderProgramID);
        glUniformMatrix4fv(glGetUniformLocation(textShaderProgramID, "proj"), 1, GL_FALSE, glm::value_ptr(projection));
        RenderText(textShaderProgramID, "Euler rotate", 350.0f, 580.0f, 0.8f, glm::vec3(1.0f, 0.0f, 0.0f));
        RenderText(textShaderProgramID, "pitch = " + to_string(rotate_x_angle), 50.0f, 550.0f, 0.8f, glm::vec3(1.0f, 0.0f, 0.0f));
        RenderText(textShaderProgramID, "yaw = " + to_string(rotate_y_angle), 320.0f, 550.0f, 0.8f, glm::vec3(1.0f, 0.0f, 0.0f));
        RenderText(textShaderProgramID, "roll = " + to_string(rotate_z_angle), 550.0f, 550.0f, 0.8f, glm::vec3(1.0f, 0.0f, 0.0f));
        if(int(rotate_x_angle) == 90 || int(rotate_y_angle) == 90 || int(rotate_z_angle) == 90 || int(rotate_x_angle) == -90 || int(rotate_y_angle) == -90 || int(rotate_z_angle) == -90){
            RenderText(textShaderProgramID, "Gimbal-locked", 300.0f, 100.0f, 0.8f, glm::vec3(1.0f, 0.0f, 0.0f));
        }
    }
    else{
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glm::mat4 projection = glm::ortho(0.0f, 800.0f, 0.0f, 600.0f);
        glUseProgram(textShaderProgramID);
        glUniformMatrix4fv(glGetUniformLocation(textShaderProgramID, "proj"), 1, GL_FALSE, glm::value_ptr(projection));
        RenderText(textShaderProgramID, "Overcome gimbal-lock using quaternions", 200.0f, 580.0f, 0.8f, glm::vec3(1.0f, 0.0f, 0.0f));
        RenderText(textShaderProgramID, "x = " + to_string(rotate_x_angle), 50.0f, 550.0f, 0.8f, glm::vec3(1.0f, 0.0f, 0.0f));
        RenderText(textShaderProgramID, "y = " + to_string(rotate_y_angle), 320.0f, 550.0f, 0.8f, glm::vec3(1.0f, 0.0f, 0.0f));
        RenderText(textShaderProgramID, "z = " + to_string(rotate_z_angle), 550.0f, 550.0f, 0.8f, glm::vec3(1.0f, 0.0f, 0.0f));
    }


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
	dataArray.push_back(SQUARE_MESH);
	dataArray.push_back(BACKPACK_MESH);
	dataArray.push_back(TEAPOT_MESH);
	generateObjectBufferMesh(dataArray);

}

// Placeholder code for the keypress
void keypress(unsigned char key, int x, int y) {
	switch (key) {
	case '1':
		rotate_x_angle += 10.0f;
        break;
	case '2':
		rotate_x_angle -= 10.0f;
		break;
	case '3':
		rotate_y_angle += 10.0f;
		break;
	case '4':
		rotate_y_angle -= 10.0f;
		break;
	case '5':
		rotate_z_angle += 10.0f;
		break;
    case '6':
        rotate_z_angle -= 10.0f;
        break;
    case 'q':
        quan = !quan;
        rotate_x_angle = 0.0f;
        rotate_y_angle = 0.0f;
        rotate_z_angle = 0.0f;
        break;
    case 's':
        rotate_x_angle = 0.0f;
        rotate_y_angle = 0.0f;
        rotate_z_angle = 0.0f;
        break;
    case 'Q':
        quan = !quan;
        rotate_x_angle = 0.0f;
        rotate_y_angle = 0.0f;
        rotate_z_angle = 0.0f;
        break;
	}
}

int main(int argc, char** argv) {

	// Set up the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(width, height);
	glutCreateWindow("Lab 1");

	// Tell glut where the display function is
	glutDisplayFunc(display);
	glutIdleFunc(updateScene);
	glutKeyboardFunc(keypress);

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
