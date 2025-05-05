using OpenTK.Windowing.Desktop;
using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;
using System;
using System.IO;

namespace RayTracing
{
    public class View : GameWindow
    {
        private int _vao;
        private int _vbo;
        private int _shaderProgram;
        private int _vertexShader, _fragmentShader;

        private readonly Vector3[] _vertices = new Vector3[]
        {
            new Vector3(-1f, -1f, 0f),
            new Vector3( 1f, -1f, 0f),
            new Vector3( 1f,  1f, 0f),
            new Vector3(-1f,  1f, 0f),
        };

        public View(int width, int height)
            : base(GameWindowSettings.Default,
                   new NativeWindowSettings() { Size = new Vector2i(width, height), Title = "Ray Tracing Lab" })
        { }

        protected override void OnLoad()
        {
            base.OnLoad();
            InitShaders();
            InitBuffers();

            GL.ClearColor(Color4.Black);
        }

        private void InitShaders()
        {
            _shaderProgram = GL.CreateProgram();
            LoadShader("Shaders/shader.vert", ShaderType.VertexShader, _shaderProgram, out _vertexShader);
            LoadShader("Shaders/shader.frag", ShaderType.FragmentShader, _shaderProgram, out _fragmentShader);

            GL.LinkProgram(_shaderProgram);

            GL.GetProgram(_shaderProgram, GetProgramParameterName.LinkStatus, out int status);
            if (status == 0)
                Console.WriteLine(GL.GetProgramInfoLog(_shaderProgram));
        }

        private void LoadShader(string path, ShaderType type, int program, out int shader)
        {
            shader = GL.CreateShader(type);
            using (StreamReader sr = new StreamReader(path))
            {
                GL.ShaderSource(shader, sr.ReadToEnd());
            }
            GL.CompileShader(shader);
            GL.AttachShader(program, shader);

            Console.WriteLine(GL.GetShaderInfoLog(shader));
        }

        private void InitBuffers()
        {
            GL.GenVertexArrays(1, out _vao);
            GL.BindVertexArray(_vao);

            GL.GenBuffers(1, out _vbo);
            GL.BindBuffer(BufferTarget.ArrayBuffer, _vbo);
            GL.BufferData(BufferTarget.ArrayBuffer, _vertices.Length * Vector3.SizeInBytes, _vertices, BufferUsageHint.StaticDraw);

            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 0, 0);

            GL.BindVertexArray(0); // Отвязываем VAO
        }

        protected override void OnRenderFrame(OpenTK.Windowing.Common.FrameEventArgs args)
        {
            base.OnRenderFrame(args);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            GL.UseProgram(_shaderProgram);
            GL.BindVertexArray(_vao); // Используем VAO вместо прямого связывания VBO

            GL.DrawArrays(PrimitiveType.TriangleFan, 0, _vertices.Length);

            SwapBuffers();
        }

        protected override void OnUnload()
        {
            base.OnUnload();
            GL.DeleteVertexArray(_vao);
            GL.DeleteBuffer(_vbo);
            GL.DeleteProgram(_shaderProgram);
            GL.DeleteShader(_vertexShader);
            GL.DeleteShader(_fragmentShader);
        }
    }
}
