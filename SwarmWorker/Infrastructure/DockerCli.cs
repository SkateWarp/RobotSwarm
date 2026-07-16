using System.ComponentModel;
using System.Diagnostics;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;

namespace SwarmWorker.Infrastructure;

public interface IDockerCli
{
    Task<DockerCommandResult> RunAsync(
        IReadOnlyList<string> arguments,
        CancellationToken cancellationToken,
        TimeSpan? timeout = null);
}

public sealed record DockerCommandResult(int ExitCode, string StandardOutput, string StandardError)
{
    public bool IsSuccess => ExitCode == 0;
}

public sealed class DockerCliException : Exception
{
    public DockerCliException(string operation, DockerCommandResult result)
        : base(
            $"Docker operation '{operation}' failed with exit code {result.ExitCode}: "
            + Truncate(result.StandardError))
    {
        Operation = operation;
        ExitCode = result.ExitCode;
    }

    public string Operation { get; }
    public int ExitCode { get; }

    private static string Truncate(string value)
    {
        var normalized = string.IsNullOrWhiteSpace(value) ? "No error output." : value.Trim();
        return normalized.Length <= 1000 ? normalized : normalized[..1000];
    }
}

public sealed class DockerCli : IDockerCli
{
    private readonly WorkerOptions _options;

    public DockerCli(IOptions<WorkerOptions> options)
    {
        _options = options.Value;
    }

    public async Task<DockerCommandResult> RunAsync(
        IReadOnlyList<string> arguments,
        CancellationToken cancellationToken,
        TimeSpan? timeout = null)
    {
        var startInfo = new ProcessStartInfo
        {
            FileName = _options.DockerExecutable,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true
        };

        foreach (var argument in arguments)
        {
            startInfo.ArgumentList.Add(argument);
        }

        using var process = new Process { StartInfo = startInfo };
        try
        {
            if (!process.Start())
            {
                throw new InvalidOperationException("Docker CLI process did not start.");
            }
        }
        catch (Win32Exception exception)
        {
            throw new InvalidOperationException(
                $"Unable to start Docker executable '{_options.DockerExecutable}'.",
                exception);
        }

        var standardOutputTask = process.StandardOutput.ReadToEndAsync(cancellationToken);
        var standardErrorTask = process.StandardError.ReadToEndAsync(cancellationToken);

        using var timeoutSource = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        timeoutSource.CancelAfter(
            timeout ?? TimeSpan.FromSeconds(_options.DockerCommandTimeoutSeconds));

        try
        {
            await process.WaitForExitAsync(timeoutSource.Token);
        }
        catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
            TryKill(process);
            throw new TimeoutException("Docker CLI operation timed out.");
        }
        catch
        {
            TryKill(process);
            throw;
        }

        return new DockerCommandResult(
            process.ExitCode,
            await standardOutputTask,
            await standardErrorTask);
    }

    private static void TryKill(Process process)
    {
        try
        {
            if (!process.HasExited)
            {
                process.Kill(entireProcessTree: true);
            }
        }
        catch (InvalidOperationException)
        {
            // The process exited between the state check and the kill request.
        }
    }
}
